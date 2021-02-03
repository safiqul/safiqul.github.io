/* -*-	Mode:C++; c-basic-offset:8; tab-width:8; indent-tabs-mode:t -*- */

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
 *	in the documentation and/or other materials provided with the distribution.
 *   -	Neither the name of Cisco Systems, Inc. nor the names of its contributors may be used to endorse or promote products derived
 *	from this software without specific prior written permission.
 * ALL MATERIALS ARE PROVIDED BY CISCO AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND NONINFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL
 * CISCO OR ANY CONTRIBUTOR BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, EXEMPLARY, CONSEQUENTIAL, OR INCIDENTAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 *
 *
 */

#ifndef ns_pie_h
#define ns_pie_h

#include "queue.h"
#include "trace.h"
#include "timer-handler.h"

#define PROB_LOW 0.85
#define PROB_HIGH 8.5
#define MIN_PKTSIZE 64
#define BURST_RESET_TIMEOUT 1.5
#define LATENCY_LOW 0.005


/*
 * Early drop parameters, supplied by user
 */
struct edp_pie {
	int mean_pktsize;	/* avg pkt size */
	double a, b;		/* parameters to PIE controller */
	double tUpdate;		/* sampling timer */
	double qdelay_ref;	/* desired queue delay */

	int byte_mode;		/* if this flag is set drop probability depends on pkt size */
	int setbit;		/* true to set congestion indication bit */
	double mark_p;		/* when p < mark_p, mark chosen packets */
				/* when p > mark_p, drop chosen packets */
	int use_mark_p;		/* use mark_p only for deciding when to drop, */

	edp_pie(): mean_pktsize(0), a(0), b(0), tUpdate(0), qdelay_ref(0), byte_mode(0), setbit(0), mark_p(0), use_mark_p(0) { }
};

/*
 * Early drop variables, maintained by PIE
 */
struct edv_pie {
	TracedDouble drop_prob;
	double accu_prob;
	double qdelay_old;

	edv_pie() : drop_prob(0), accu_prob(0), qdelay_old(0) { }
};

class LinkDelay;
class PIEQueue;
class DocsisLink;

class PIECalcTimer : public TimerHandler {
public:
	PIECalcTimer(PIEQueue *a) : TimerHandler() { a_ = a; }
	virtual void expire(Event *e);
protected:
	PIEQueue *a_;
};

enum burst_state_t {
	NO_BURST,
	IN_BURST,
	IN_BURST_PROTECTING,
};

class PIEQueue : public Queue {
 friend class PIECalcTimer;
public:
	PIEQueue(const char * = "Drop");
protected:
	int command(int argc, const char*const* argv);
	void enque(Packet* pkt);
	Packet* deque();
	void reset();
	bool drop_early(Packet* pkt, int qlen, int qlim);
	void calculate_p();

	PIECalcTimer CalcTimer;
	double sUpdate_;	/* Time to start the update timer*/

	PacketQueue *q_;	/* underlying FIFO queue */
	NsObject* de_drop_;	/* drop_early target */

	//added to be able to trace EDrop Objects - ratul
	//the other events - forced drop, enque and deque are traced by a different mechanism.
	Trace* EDTrace;		/* early drop trace */
	char traceType[20];	/* the preferred type for early drop trace. */
				/* better be less than 19 chars long */
	Tcl_Channel tchan_;	/* place to write trace records */
	TracedInt curq_;	/* current qlen seen by arrivals */
	void trace(TracedVar*);	/* routine to write trace records */

	edp_pie edp_;		/* early-drop params */
	edv_pie edv_;		/* early-drop variables */

	double burst_allowance_; /* current max burst size that is allowed before random drops kick in*/
	double max_burst_;	/* maximum burst allowed before random early dropping kicks in*/
	int burst_reset_;
	burst_state_t burst_state_;

	bool in_measurement;	/* indicate whether we are in a measurement cycle */
	int dq_threshold;	/*threshold that needs to be across before a sample of the dequeue rate is measured */
	double avg_dq_rate;	/*time averaged dequeue rate*/
	double dq_start;	/*the start timestamp of current measurement cycle*/
	int dq_count;		/*number of bytes departed since current measurement cycle starts*/

	bool Docsis_mode_;	/* 1: in docsis mode; 0: normal mode*/

	DocsisLink* link_;	/* For latency estimation */
};
#endif
