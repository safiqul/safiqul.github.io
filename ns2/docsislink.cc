/*
 * DocsisLink - A delay link that models DOCSIS Cable Modem upstream link
 */

/*
 * Copyright (c) 2012-2013 Cable Television Laboratories, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The names of the authors may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * Alternatively, provided that this notice is retained in full, this
 * software may be distributed under the terms of the GNU General
 * Public License ("GPL") version 2, in which case the provisions of the
 * GPL apply INSTEAD OF those given above.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * Authors:
 *   Greg White <g.white@cablelabs.com>
 *   Joey Padden <j.padden@cablelabs.com>
 *   Takashi Hayakawa <t.hayakawa@cablelabs.com>
 */

#include "docsislink.h"
#include "random.h"
#include <assert.h>
#include <iostream>

static class DocsisLinkClass : public TclClass {
public:
    DocsisLinkClass() : TclClass("DocsisLink") {}
    TclObject* create(int /* argc */, const char*const* /* argv */) {
        return (new DocsisLink);
    }
} class_docsislink_link;

DocsisLink::DocsisLink() : map_timer_(this, false), grant_timer_(this, true)
{
    initialized_ = false;

    feederq_ = NULL;
    qh_ = NULL;

    bind("maxgrant_", &maxgrant_);
    bind("mgvar_", &mgvar_);
    bind("mapint_", &mapint_);
    bind("rate_",  &rate_);
    bind("peakrate_", &peakrate_);
    bind("bucket_", &bucket_);
    bind("peakbucket_", &peakbucket_);

    map_timer_.resched(mapint_);
}

int DocsisLink::command(int argc, const char*const* argv)
{
    Tcl& tcl = Tcl::instance();

    if (argc == 3) {
        if (strcmp(argv[1], "feederq") == 0) {
            const char *feederq_id = argv[2];
            if (feederq_id[0] == '0') {
                feederq_ = 0;
                return (TCL_OK);
            }
            feederq_ = (Queue*) TclObject::lookup(feederq_id);
            if (feederq_ == 0) {
                tcl.resultf("no such object %s", feederq_id);
                return (TCL_ERROR);
            }

            feederq_->block();
            tcl.evalf("%s set unblock_on_resume_ false", feederq_id);
            return (TCL_OK);
        }
    }

    return LinkDelay::command(argc, argv);
}

void DocsisLink::reset()
{
    initialized_ = false;
    LinkDelay::reset();
}

void DocsisLink_Timer::expire(Event*)
{
    if (isGrantTimer_)
        doc_->handleGrantTimer();
    else
        doc_->handleMapTimer();
}

////////////////////////////////////////////////////////////////

/*
 * Called at each MAP interval.  Gets next grant and retrieve token count.
 */
void DocsisLink::handleMapTimer()
{
    initializeIfNecessary();

    // Schedule the next MAP.
    map_timer_.resched(mapint_);

    // Receive new grant from CMTS, simulating the variability from
    // congestion.
    int w = mgvar_ * maxgrant_ / 100;
    int r = rand() % (2 * w) - w;
    int nextmax = maxgrant_ + r;
    int grant = min(req_[0], nextmax);

    // Move previous requests.
    req_[0] = max(0, req_[0] - grant) + req_[1];
    req_[1] = 0;

    // Apply the grant to tokens.
    tokens_ += grant;

    if (tokens_ == 0) {
        // If no grant, make a contention request.
        makeRequest();
    } else {
        // If we have a grant, schedule use of it.
        double latest_grant = mapint_ - transmissionTime(tokens_ - frag_sent_bytes_);
        double grant_time = Random::uniform(max(0.0, latest_grant));
        grant_timer_.resched(grant_time);
    }
}

/*
 * Called at the start of each grant.  Transmits a segment containing as many
 * packets as will fit.
 */
void DocsisLink::handleGrantTimer()
{
    // make a piggybacked request in the segment header
    makeRequest();

    if (frag_pkt_ == NULL) {
        // Fragmentation not in progress - ask for a new packet
        askForNextPacket(qh_, 0);
    } else {
        // Fragmentation in progress
        int frag_pkt_size = HDR_CMN(frag_pkt_)->size_;
        if (tokens_  < frag_pkt_size) {
            // Not enough tokens - continue fragmentation
            frag_sent_bytes_ = tokens_;
        } else {
            // Enough tokens - finish this packet within this grant
            double pkt_departure = transmissionTime(frag_pkt_size - frag_sent_bytes_);
            assert(pkt_departure > 0);
            Scheduler::instance().schedule(target_, frag_pkt_, delay_ + pkt_departure);
            tokens_ -= frag_pkt_size;
            frag_pkt_ = NULL;
            frag_sent_bytes_ = 0;

            askForNextPacket(qh_, pkt_departure);
        }
    }
}

/*
 * Receives a new packet from upstream queue.  Only gets called by the
 * upstream queue in response to the queue being unblocked by
 * askForNextPacket().  If enough tokens_ for the packet, send it to target_.
 * Otherwise, hold the packet for fragmentation.
 */
void DocsisLink::recv(Packet* p, Handler* h)
{
    assert(frag_pkt_ == NULL);
    assert(frag_sent_bytes_ == 0);

    int pkt_size = HDR_CMN(p)->size_;
    if (tokens_ >= pkt_size) {
        // Fits in current grant - send it entirely
        double pkt_departure = transmissionTime(pkt_size);
        Scheduler::instance().schedule(target_, p, delay_ + pkt_departure);
        tokens_ -= pkt_size;    // Spend the tokens for this packet.
        askForNextPacket(h, pkt_departure);
    } else {
        // Doesn't fit in current grant - start fragmentation
        frag_pkt_ = p;
        frag_sent_bytes_ = tokens_;
        qh_ = h;
    }
}

/*
 * Generates request based on current queue length minus already requested
 * bytes, capped by rate shaper tokens.
 */
void DocsisLink::makeRequest(void)
{
    // Refill rate shaper tokens.
    double now = Scheduler::instance().clock();
    double elapsed = now - last_update_time_;
    msrtokens_ += elapsed * rate_ / 8;
    if (msrtokens_ > bucket_)
        msrtokens_ = bucket_;
    peaktokens_ = elapsed * peakrate_ / 8;
    last_update_time_ = now;

    // Determine the new request size.
    int curq = feederq_->byteLength() + (frag_pkt_ ? HDR_CMN(frag_pkt_)->size_ : 0);
    req_[1] = max(0, curq - req_[0] - tokens_);
    req_[1] = min(min(req_[1], msrtokens_), peaktokens_);
    msrtokens_ -= req_[1];
}

void DocsisLink::askForNextPacket(Handler* h, double howSoon)
{
    assert(frag_pkt_ == NULL);
    assert(frag_sent_bytes_ == 0);

    if (tokens_ > 0) {
        if (feederq_->byteLength() > 0) {
            if (howSoon > 0) {
                assert(h != NULL);
                Scheduler::instance().schedule(h, &intr_, howSoon);
            } else {
                feederq_->resume();
            }
        } else {
            // Queue is empty, but we still have tokens.
            // Dump the tokens.  This can happen when feederq_ drops its packets.
            cout << "DocsisLink: Dumping " << tokens_ << " tokens." << endl;
            tokens_ = 0;
        }
    }
}

void DocsisLink::initializeIfNecessary()
{
    if (!initialized_) {
       initialized_ = true;
       tokens_ = 0;
       msrtokens_ = bucket_;
       peaktokens_ = 0;
       last_update_time_ = Scheduler::instance().clock();
       req_[0] = 0;
       req_[1] = 0;

       frag_pkt_ = NULL;
       frag_sent_bytes_ = 0;
    }
}

////////////////////////////////////////////////////////////////

double DocsisLink::expectedDelay(int queue_size) {
    double latency;
    if (queue_size <= msrtokens_) {
        latency = 8 * queue_size / peakrate_;
    } else {
        latency = 8 * ((queue_size - msrtokens_) / rate_ +  msrtokens_ / peakrate_);
    }
    return latency;
}
