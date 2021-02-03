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

#ifndef ns_docsislink_h
#define ns_docsislink_h

#include "delay.h"
#include "timer-handler.h"

class DocsisLink;

class DocsisLink_Timer : public TimerHandler {
public:
    DocsisLink_Timer(DocsisLink *d, int isGrant) : TimerHandler() {
        doc_ = d;
        isGrantTimer_ = isGrant;
    }

protected:
    virtual void expire(Event *e);
    DocsisLink *doc_;
    int isGrantTimer_;
};

class DocsisLink : public LinkDelay {
public:
    DocsisLink();
    void recv(Packet* p, Handler*);
    void handleMapTimer();
    void handleGrantTimer();

    inline double transmissionTime(int bytes) {
        return 8 * double(bytes) / bandwidth_;
    }

    double expectedDelay(int queue_size);

protected:
    int command(int argc, const char*const* argv);
    void reset();

    Queue* feederq_;
    Handler* qh_; //place to save blocked upstream q's handler

    void makeRequest();
    void askForNextPacket(Handler* qh, double howSoon);
    void initializeIfNecessary();

    // DOCSIS configuration variables
    int maxgrant_;      // To model congestion on the DOCSIS link by limiting the bytes
                        // per map that the flow can recieve
    int mgvar_;         // "MaxGrant Variability", to simulate random grant sizing per MAP
                        // (RANGE: 0 - 100)
    double mapint_;     // The MAP interval
    double rate_;       // MSR token bucket rate
    double peakrate_;   // peak token rate
    int bucket_;        // MSR bucket depth
    int peakbucket_;    // peak bucket depth

    // Internal variables
    bool initialized_;
    DocsisLink_Timer map_timer_;
    DocsisLink_Timer grant_timer_;

    int tokens_;        // accumulated/granted tokens
    int msrtokens_;     // accumulated Max Sustained Rate tokens
    int peaktokens_;    // accumulated peak tokens
    double last_update_time_;

    int req_[2];        // [0]: currently-outstanding request; [1]: new request

    Packet* frag_pkt_;  // Fragmented packet across multiple grants
    int frag_sent_bytes_; // # of bytes Fragment
};

#endif
