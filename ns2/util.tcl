if {$argc < 4} {
    puts "error! usage: ns util.tcl tcptype queuetype capacity RTT"
    exit;
}

#---------------------------------------------------------
# Simulation parameters:
#
# tcptype: Type of tcp sender (reno, cubic)
set tcptype [lindex $argv 0]
#
# qtype: Type of queue (CoDel, PIE, RED, DropTail, ..)
set qtype [lindex $argv 1]
#
# bc: Bottleneck capacity
set bc "[lindex $argv 2]Mb"
#
# linkDelay - half the RTT given by the user
# NOTE:
# to better match the TEACUP testbed, add 0.08ms to linkDelay
set linkdelay "[expr [lindex $argv 3]/2]ms"

#
#
set duration 160.0
#
#
#--------------------------------------------------------

set ns [new Simulator]

#Open the output files

set nf [open out.nam w]
#$ns namtrace-all $nf
set f [open "| perl stats_cut.pl 10 tcp 1" w]
#set f [open out.tr w]
$ns trace-all $f

set tcpf [open tcp.tr w]

#Create 2 nodes
set n0 [$ns node]
set n1 [$ns node]


#Queue/RED set targetdelay_ 0.005
#Queue/RED set q_weight_ -1
#Queue/RED set thresh_ 0
#Queue/RED set maxthresh_ 0
#Queue/RED set setbit_ false

Queue/CoDel set target_ [delay_parse 5ms]
Queue/CoDel set interval_ [delay_parse 100ms]
# DO ECN?
Queue/CoDel set setbit_ false

Queue/sfqCoDel set target_ [delay_parse 5ms]
Queue/sfqCoDel set interval_ [delay_parse 100ms]
# DO ECN?
Queue/sfqCoDel set setbit_ false

Queue/PIE set mark_p_ 0.1
#this disables marking only up to mark_p (default 10%) loss probability
Queue/PIE set use_mark_p_ false
Queue/PIE set tUpdate_ 30ms
Queue/PIE set queue_in_bytes_ false
Queue/PIE set burst_allowance_ 0.1
# DO ECN?
Queue/PIE set setbit_ false
# pie parameters: qdelay_ref_ is target; a_ is alpha; b_ is beta


#Connect the nodes
$ns duplex-link $n0 $n1 $bc $linkdelay $qtype
#CoDel and PIE default:
$ns queue-limit $n0 $n1 1000


set tcp0 [new Agent/TCP/Linux]
$tcp0 set window_ 100000
$tcp0 set timestamps_ true
# this includes 10 bytes for timestamps
$tcp0 set packetSize_ 1450
$tcp0 set fid_ 0
$tcp0 set ecn_ 1
$ns attach-agent $n0 $tcp0

$tcp0 set ss_beta [expr 0.5*1024]
$tcp0 set beta [expr 0.5*1024]
$tcp0 set eem_ss_beta [expr 0.5*1024]
$tcp0 set eem_beta [expr 0.5*1024]




set sink0 [new Agent/TCPSink/Sack1]
$sink0 set ts_echo_rfc1323_ true
$ns attach-agent $n1 $sink0


$ns connect $tcp0 $sink0

# Create an FTP traffic source and attach it to tcp0
set ftp0 [new Application/FTP]
$ftp0 attach-agent $tcp0


# log cwnd, ssthresh, rtt
  proc plottcpvars {tcpSource outfile} {
     global ns

     set now [$ns now]
     set cwnd [$tcpSource set cwnd_]
     set ssthresh [$tcpSource set ssthresh_]
     set rtt [$tcpSource set rtt_]
     puts $outfile "$now $cwnd $ssthresh $rtt"
     $ns at [expr $now+0.1] "plottcpvars $tcpSource $outfile"
  }


#Define a 'finish' procedure
proc finish {} {
	global nf f tcpf
	#Close the output files
	close $nf
        close $f
        close $tcpf
      exit 0
}

#$ns at 0.0 "plottcpvars $tcp0 $tcpf"
$ns at 0 "$tcp0 select_ca $tcptype"
$ns at 0 "$tcp0 set_ca_default_param cubic hystart 1"

#Start the traffic sources
$ns at 0.0 "$ftp0 start"

#Call the finish procedure after 600 seconds simulation time
$ns at 310.0 "finish"

#Run the simulation
$ns run
