/* -*- Mode:C++; c-basic-offset:8; tab-width:8; indent-tabs-mode:t -*- */
/*
 * Proportional Integral Controller - Enhanced (PIE) AQM Implementation
 *
 * Authors of the code:
 * Preethi Natarajan (prenatar@cisco.com)
 * Rong Pan (ropan@cisco.com)
 * Chiara Piglione (cpiglion@cisco.com)
 * Greg White (g.white@cablelabs.com)
 * Takashi Hayakawa (t.hayakawa@cablelabs.com)

 * Copyright (c) 2013-2014, Cisco Systems, Inc.
 * Portions Copyright (c) 2013-2014 Cable Television Laboratories, Inc.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following
 * conditions are met:
 *   -	Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *   -	Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer
 *      in the documentation and/or other materials provided with the distribution.
 *   -	Neither the name of Cisco Systems, Inc. nor the names of its contributors may be used to endorse or promote products derived
 *      from this software without specific prior written permission.
 * ALL MATERIALS ARE PROVIDED BY CISCO AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND NONINFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL
 * CISCO OR ANY CONTRIBUTOR BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, EXEMPLARY, CONSEQUENTIAL, OR INCIDENTAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 */

#include <math.h>
#include <sys/types.h>
#include "config.h"
#include "template.h"
#include "random.h"
#include "flags.h"
#include "delay.h"
#include "docsislink.h"
#include "pie.h"

static class PIEClass : public TclClass {
public:
	PIEClass() : TclClass("Queue/PIE") {}
	TclObject* create(int argc, const char*const* argv) {
		if (argc == 5)
			return (new PIEQueue(argv[4]));
		else
			return (new PIEQueue("Drop"));
	}
} class_pie;


PIEQueue::PIEQueue(const char * trace) : CalcTimer(this), sUpdate_(0),
					 de_drop_(NULL), EDTrace(NULL), tchan_(0), curq_(0),
					 edp_(), edv_(), burst_allowance_(0), max_burst_(0),
					 burst_reset_(0), burst_state_(NO_BURST),
					 dq_threshold(0), avg_dq_rate(0.0),dq_count(-1),
					 Docsis_mode_(0), link_(NULL)
{
	if (strlen(trace) >= 20) {
		printf("trace type too long - allocate more space to traceType in pie.h and recompile\n");
		exit(1);
	}
	strcpy(traceType, trace);

	bind("a_", &edp_.a);
	bind("b_", &edp_.b);
	bind_time("tUpdate_", &edp_.tUpdate);
	bind_time("sUpdate_", &sUpdate_);
	bind("qdelay_ref_", &edp_.qdelay_ref);
	bind("mean_pktsize_", &edp_.mean_pktsize);
	bind("burst_allowance_", &max_burst_);

	/* Traced values */
	bind("prob_", &edv_.drop_prob);
	bind("curq_", &curq_);

	bind("dq_threshold_", &dq_threshold);
	bind_bool("bytes_", &edp_.byte_mode);

	bind_bool("setbit_", &edp_.setbit);	    // mark instead of drop
	bind("mark_p_", &edp_.mark_p);
	bind_bool("use_mark_p_", &edp_.use_mark_p);

	q_ = new PacketQueue();
	pq_ = q_;
	reset();
}

void PIEQueue::reset()
{
	edv_.drop_prob = 0;
	edv_.accu_prob = 0.0;
	curq_ = 0;

	if (Docsis_mode_ && !edp_.byte_mode) {
		fprintf(stderr, "PIE: DOCSIS mode but not byte mode\n");
	}

	CalcTimer.resched(sUpdate_);
	Queue::reset();
}


////////////////
// Data Path
////////////////

void PIEQueue::enque(Packet* pkt)
{
	int qlen = q_->byteLength();
	curq_ = qlen;
	int qlim = qlim_ * edp_.mean_pktsize;

	if (qlen >= qlim) {
		/* Forced drop: reactive to full que */
		drop(pkt);
		edv_.accu_prob = 0;
	} else if (drop_early(pkt, qlen, qlim)) {
		/* Unforced drop: proactive */
		drop(pkt);
	} else {
		/* No drop */
		q_->enque(pkt);
	}
}

bool PIEQueue::drop_early(Packet* pkt, int qlen, int qlim)
{

	if (burst_allowance_ > 0) {
		/* If there is still burst_allowance left, skip random early drop. */
		return false;
	}


	if (edv_.drop_prob == 0.0) {
		edv_.accu_prob = 0.0;
	}

	if (burst_state_ == NO_BURST) {
		if (Docsis_mode_) {
			if ( qlen < qlim / 3)
				return false;
			else
				burst_state_ = IN_BURST;
		} else {
			burst_state_ = IN_BURST_PROTECTING;
			burst_allowance_ = max_burst_;
		}
	}


	double p = edv_.drop_prob;
	int packet_size = hdr_cmn::access(pkt)->size();

	if (edp_.byte_mode) p = p*packet_size / edp_.mean_pktsize;

	if (Docsis_mode_) {
		if (edp_.byte_mode) {
			p = min(p, PROB_LOW);
		}
		edv_.accu_prob += p;
	}

	int early_drop = 1;

	double u = Random::uniform();
	if (edv_.qdelay_old < 0.5 * edp_.qdelay_ref && edv_.drop_prob < 0.2) {
		return false;
	} else if (qlen <= 2 * edp_.mean_pktsize) {
		return false;
	}

	if (Docsis_mode_) {
		if (edv_.accu_prob < PROB_LOW) {
			// Avoid dropping too fast due to bad luck of coin tosses
			early_drop = 0;
		} else if (edv_.accu_prob > PROB_HIGH) {
			// Avoid dropping too slow due to bad luck of coin tosses
			early_drop = 1;
		} else {
			if (u > p) {
				early_drop = 0;
			}
		}
	} else {
		if (u > p) {
			early_drop = 0;
		} else {
			/* Try ECN first */
			hdr_flags* hf = hdr_flags::access(pkt);

			/* 1. No ECN in DOCSIS mode (per DOCSIS std)
			 * 2. if use_mark_p set to true then
			 *	ECN mark only when prob < mark_p
			 *	and prob < mark_p then drop
			 *	This logic is similar to RED queue
			 */

			if (edp_.setbit && hf->ect() &&
				(!edp_.use_mark_p || (p < edp_.mark_p))) {
				hf->ce() = 1;   // mark Congestion Experienced bit
				early_drop = 0;     // no drop
			} // else packet dropped (early_drop = 1)
		}
	}

	if (early_drop == 0) return false;

	edv_.accu_prob = 0.0;
	if (Docsis_mode_) {
		if (burst_state_ == IN_BURST) {
			burst_state_ = IN_BURST_PROTECTING;
			burst_allowance_ = max_burst_;
		}
	}

	return true;
}

Packet* PIEQueue::deque()
{
	Packet *p;
	p = q_->deque();
	double now = Scheduler::instance().clock();
	int pkt_size = (p != NULL) ? hdr_cmn::access(p)->size() : 0;

	if (!Docsis_mode_) {
		// if not in a measurement cycle and the queue has built up to dq_threshold,
		// start the measurement cycle
		if ( (q_->byteLength() >= (dq_threshold)) && (in_measurement == 0) ) {
			dq_start = now;
			dq_count = 0;
			in_measurement = 1;
		}

		if (in_measurement == 1) {
			dq_count += pkt_size;
			// done with a measurement cycle
			if (dq_count >= (dq_threshold)) {
				double tmp = now - dq_start;

				if (avg_dq_rate == 0) avg_dq_rate = dq_count/tmp;
				else avg_dq_rate = 0.5*avg_dq_rate+0.5*dq_count/tmp;

				// restart a measurement cycle if there is enough data
				if (q_->byteLength() > (dq_threshold)) {
					dq_start = now;
					dq_count = 0;
					in_measurement = 1;
				} else {
					dq_count = 0;
					in_measurement = 0;
				}
			}
		} else {
			// if not in a measurement cycle and the queue has built up to dq_threshold,
			// start the measurement cycle
			if ( (q_->length() >= dq_threshold) && (in_measurement == 0) ) {
				dq_start = now;
				dq_count = 0;
				in_measurement = 1;
			}

			//in a measurement cycle
			if (in_measurement == 1) {
				dq_count ++;
				// done with a measurement cycle
				if (dq_count >= dq_threshold) {
					double tmp = now - dq_start;
					// time averaging deque rate, start off with the
					// current value, otherwise, average it.
					if (avg_dq_rate == 0) avg_dq_rate = dq_count/tmp;
					else avg_dq_rate = 0.5*avg_dq_rate+0.5*dq_count/tmp;

					// restart a measurement cycle if there is enough data
					if (q_->length() > dq_threshold) {
						dq_start = now;
						dq_count = 0;
						in_measurement = 1;
					} else {
						dq_count = 0;
						in_measurement = 0;
					}
				}
			}
		}
	}

	curq_ = q_->byteLength(); // helps to trace queue during arrival, if enabled
	return (p);
}




////////////////
// Control Path
////////////////

void PIEQueue::calculate_p()
{
	double qdelay;
	bool missing_init_flag = false;

	if (Docsis_mode_) {
		qdelay = link_->expectedDelay(q_->byteLength());
	} else {
		if (avg_dq_rate > 0){
			qdelay = q_->byteLength()/avg_dq_rate;
		} else {
			qdelay = 0.0;
			missing_init_flag = true;
		}
	}

	if (burst_allowance_ > 0) {
		edv_.drop_prob = 0;
	} else {
		double p = edp_.a * (qdelay - edp_.qdelay_ref)
			+ edp_.b * (qdelay - edv_.qdelay_old);

		int Docsis_continue = 0;

		if (Docsis_mode_) {
			if (edv_.drop_prob < 0.000001) { // Cover extremely low drop prob scenarios
				p /= 2048;
			} else if (edv_.drop_prob < 0.00001) {
				p /= 512;
			} else if (edv_.drop_prob < 0.0001) {
				p /= 128;
			} else {
				Docsis_continue = 1;
			}
		}

		if (Docsis_mode_ == 0 || Docsis_continue) {
			if (edv_.drop_prob < 0.001) {
				p /= 32;
			} else if (edv_.drop_prob < 0.01) {
				p /= 8;
			} else if (edv_.drop_prob < 0.1) {
				p /= 2;
			} else if (edv_.drop_prob < 1) {
				p /= 0.5;
			} else if (edv_.drop_prob < 10) {
				p /= 0.125;
			} else {
				p /= 0.03125;
			}

			if ((edv_.drop_prob >= 0.1) && (p > 0.02)) {
				p = 0.02;
			}
		}
		p += edv_.drop_prob;

		/* For non-linear drop in prob */
		if (Docsis_mode_ == 0) {
			if (qdelay == 0 && edv_.qdelay_old == 0) {
				p *= 0.98; // or 63/64
			} else if (qdelay > 0.2) {
				p += 0.02;
			}
		} else {
			if (qdelay < LATENCY_LOW && edv_.qdelay_old < LATENCY_LOW) {
				p *= 0.98; // or 63/64
			} else if (qdelay > 0.2) {
				p += 0.02;
			}
		}

		edv_.drop_prob = max(0, p);
		if (Docsis_mode_) edv_.drop_prob = min(edv_.drop_prob, PROB_LOW * edp_.mean_pktsize/MIN_PKTSIZE);
	}

	if (burst_allowance_ < edp_.tUpdate) {
		burst_allowance_ = 0;
	} else {
		burst_allowance_ -= edp_.tUpdate;
	}


	int burst_reset_limit = BURST_RESET_TIMEOUT / edp_.tUpdate;

	if (!Docsis_mode_) {
		if( (qdelay < 0.5*edp_.qdelay_ref) && (edv_.qdelay_old < (0.5*edp_.qdelay_ref) ) &&
		    (edv_.drop_prob == 0) && !missing_init_flag ) {
			dq_count = -1;
			avg_dq_rate = 0.0;
		}
	}

	if (qdelay < 0.5 * edp_.qdelay_ref
	    && edv_.qdelay_old < 0.5 * edp_.qdelay_ref
	    && edv_.drop_prob == 0
	    && burst_allowance_ == 0) {
		if (burst_state_ == IN_BURST_PROTECTING) {
			burst_state_ = IN_BURST;
			burst_reset_ = 0;
		} else if (burst_state_ == IN_BURST) {
			burst_reset_++;
			if (burst_reset_ > burst_reset_limit) {
				burst_reset_ = 0;
				burst_state_ = NO_BURST;
			}
		}
	} else if (burst_state_ == IN_BURST) {
		burst_reset_ = 0;
	}

	edv_.qdelay_old = qdelay;

	CalcTimer.resched(edp_.tUpdate);
}

void PIECalcTimer::expire(Event *)
{
	a_->calculate_p();
}


////////////////
// Tcl interface
////////////////

int PIEQueue::command(int argc, const char*const* argv)
{
	Tcl& tcl = Tcl::instance();
	if (argc == 2) {
		if (strcmp(argv[1], "reset") == 0) {
			reset();
			return (TCL_OK);
		}
		if (strcmp(argv[1], "early-drop-target") == 0) {
			if (de_drop_ != NULL)
				tcl.resultf("%s", de_drop_->name());
			return (TCL_OK);
		}
		if (strcmp(argv[1], "edrop-trace") == 0) {
			if (EDTrace != NULL) {
				tcl.resultf("%s", EDTrace->name());
			}
			else {
				tcl.resultf("0");
			}
			return (TCL_OK);
		}
		if (strcmp(argv[1], "trace-type") == 0) {
			tcl.resultf("%s", traceType);
			return (TCL_OK);
		}
	}
	else if (argc == 3) {
		// attach a file for variable tracing
		if (strcmp(argv[1], "attach") == 0) {
			int mode;
			const char* id = argv[2];
			tchan_ = Tcl_GetChannel(tcl.interp(), (char*)id, &mode);
			if (tchan_ == 0) {
				tcl.resultf("PIE: trace: can't attach %s for writing", id);
				return (TCL_ERROR);
			}
			return (TCL_OK);
		}
		// tell PIE about link stats
		if (strcmp(argv[1], "link") == 0) {
			TclObject* del = TclObject::lookup(argv[2]);
			if (del == 0) {
				tcl.resultf("PIE: non LinkDelay object %s", argv[2]);
				return(TCL_ERROR);
			}

			tcl.evalf("%s info class", argv[2]);
			const char* classname = tcl.result();
			if (strcmp(classname, "DocsisLink") == 0) {
				link_ = (DocsisLink*) del;
				Docsis_mode_ = true;
			}

			return (TCL_OK);
		}
		if (strcmp(argv[1], "early-drop-target") == 0) {
			NsObject* p = (NsObject*) TclObject::lookup(argv[2]);
			if (p == 0) {
				tcl.resultf("no object %s", argv[2]);
				return (TCL_ERROR);
			}
			de_drop_ = p;
			return (TCL_OK);
		}
		if (strcmp(argv[1], "edrop-trace") == 0) {
			Trace* t  = (Trace*) TclObject::lookup(argv[2]);
			if (t == 0) {
				tcl.resultf("no object %s", argv[2]);
				return (TCL_ERROR);
			}
			EDTrace = t;
			return (TCL_OK);
		}
		if (!strcmp(argv[1], "packetqueue-attach")) {
			delete q_;
			if (!(q_ = (PacketQueue*) TclObject::lookup(argv[2])))
				return (TCL_ERROR);
			else {
				pq_ = q_;
				return (TCL_OK);
			}
		}
	}
	return (Queue::command(argc, argv));
}

void PIEQueue::trace(TracedVar* v)
{
	char wrk[500];
	const char *p;

	if (((p = strstr(v->name(), "prob")) == NULL) &&
	    ((p = strstr(v->name(), "curq")) == NULL)) {
		fprintf(stderr, "PIE:unknown trace var %s\n", v->name());
		return;
	}
	if (tchan_) {
		double t = Scheduler::instance().clock();
		// XXX: be compatible with nsv1 PI trace entries
		if (*p == 'c') {
			sprintf(wrk, "Q %g %d\n", t, int(*((TracedInt*) v)));
		} else {
			sprintf(wrk, "%c %g %g\n", *p, t, double(*((TracedDouble*) v)));
		}
		int n = strlen(wrk);
		(void) Tcl_Write(tchan_, wrk, n);
	}
}
