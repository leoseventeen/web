#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "sr.h"
/* ******************************************************************
   Go Back N protocol.  Adapted from J.F.Kurose
   ALTERNATING BIT AND GO-BACK-N NETWORK EMULATOR: VERSION 1.2

   Network properties:
   - one way network delay averages five time units (longer if there
   are other messages in the channel for GBN), but can be larger
   - packets can be corrupted (either the header or the data portion)
   or lost, according to user-defined probabilities
   - packets will be delivered in the order in which they were sent
   (although some can be lost).

   Modifications:
   - removed bidirectional GBN code and other code not used by prac.
   - fixed C style to adhere to current programming style
   - added GBN implementation
**********************************************************************/

#define RTT  16.0       /* round trip time.  MUST BE SET TO 16.0 when submitting assignment */
#define WINDOWSIZE 6    /* the maximum number of buffered unacked packet
                          MUST BE SET TO 6 when submitting assignment */
#define SEQSPACE 7      /* the min sequence space for GBN must be at least windowsize + 1 */
#define NOTINUSE (-1)   /* used to fill header fields that are not being used */

/* generic procedure to compute the checksum of a packet.  Used by both sender and receiver
   the simulator will overwrite part of your packet with 'z's.  It will not overwrite your
   original checksum.  This procedure must generate a different checksum to the original if
   the packet is corrupted.
*/

/* Compute checksum over seqnum, acknum, and payload */
static int ComputeChecksum(struct pkt p) {
    int sum = p.seqnum + p.acknum;
    for (int i = 0; i < 20; i++) sum += (unsigned char)p.payload[i];
    return sum;
}
static bool IsCorrupted(struct pkt p) {
    return p.checksum != ComputeChecksum(p);
}

/* Sender (A) state */
static struct pkt  A_window[WINDOWSIZE];
static bool        A_acked[WINDOWSIZE];
static bool        A_timer_running[WINDOWSIZE];
static int         A_base;
static int         A_nextseq;

/* Receiver (B) state */
static struct pkt  B_buffer[WINDOWSIZE];
static bool        B_received[WINDOWSIZE];
static int         B_base;

/* A_init: called once at start */
void A_init(void) {
    A_base    = 0;
    A_nextseq = 0;
    for (int i = 0; i < WINDOWSIZE; i++) {
        A_acked[i]         = false;
        A_timer_running[i] = false;
    }
}

/* A_output: called from layer5 to send message */
void A_output(struct msg message) {
    /* if window not full */
    if (A_nextseq < A_base + WINDOWSIZE) {
        int seq = A_nextseq % SEQSPACE;
        struct pkt p;
        p.seqnum  = seq;
        p.acknum  = NOTINUSE;
        memcpy(p.payload, message.data, 20);
        p.checksum = ComputeChecksum(p);

        /* buffer and send */
        A_window[seq]  = p;
        A_acked[seq]   = false;
        tolayer3(A, p);

        /* start timer for this packet if not running */
        if (!A_timer_running[seq]) {
            starttimer(A, RTT);
            A_timer_running[seq] = true;
        }
        A_nextseq++;
    } else {
        /* window full: drop message */
        if (TRACE > 0) printf("----A: window full, drop msg\n");
        window_full++;
    }
}

/* A_input: called from layer3 when ACK arrives */
void A_input(struct pkt packet) {
    if (IsCorrupted(packet)) return;
    int ack = packet.acknum;
    /* check if ack corresponds to any unacked pkt in window */
    for (int i = A_base; i < A_nextseq; i++) {
        if (ack == (i % SEQSPACE)) {
            int idx = ack;
            if (!A_acked[idx]) {
                A_acked[idx] = true;
                total_ACKs_received++;
                /* stop timer for this packet */
                if (A_timer_running[idx]) {
                    stoptimer(A);
                    A_timer_running[idx] = false;
                }
                /* slide window base forward for contiguous ACKs */
                while (A_acked[A_base % SEQSPACE]) {
                    A_acked[A_base % SEQSPACE] = false;
                    A_base++;
                }
            }
            break;
        }
    }
}

/* A_timerinterrupt: called when A's timer expires */
void A_timerinterrupt(void) {
    if (TRACE > 0) printf("----A: timeout, resending unacked pkts\n");
    /* retransmit all unacked in window */
    for (int i = A_base; i < A_nextseq; i++) {
        int idx = i % SEQSPACE;
        if (!A_acked[idx]) {
            tolayer3(A, A_window[idx]);
            packets_resent++;
            /* restart timer */
            if (A_timer_running[idx]) stoptimer(A);
            starttimer(A, RTT);
            A_timer_running[idx] = true;
        }
    }
}

/* B_init: receiver initialization */
void B_init(void) {
    B_base = 0;
    for (int i = 0; i < WINDOWSIZE; i++) {
        B_received[i] = false;
    }
}

/* B_input: called from layer3 when data arrives */
void B_input(struct pkt packet) {
    struct pkt ackpkt;
    ackpkt.seqnum = NOTINUSE;
    ackpkt.acknum = packet.seqnum;
    memset(ackpkt.payload, 0, 20);
    ackpkt.checksum = ComputeChecksum(ackpkt);

    /* check window membership */
    bool inWindow = false;
    for (int i = B_base; i < B_base + WINDOWSIZE; i++) {
        if (packet.seqnum == (i % SEQSPACE)) { inWindow = true; break; }
    }
    if (!IsCorrupted(packet) && inWindow) {
        int idx = packet.seqnum;
        if (!B_received[idx]) {
            B_buffer[idx]   = packet;
            B_received[idx] = true;
        }
        /* send ACK */
        tolayer3(B, ackpkt);
    } else {
        /* out-of-window or corrupted: resend last in-order ACK */
        int last = (B_base == 0 ? SEQSPACE-1 : (B_base-1) % SEQSPACE);
        ackpkt.acknum = last;
        ackpkt.checksum = ComputeChecksum(ackpkt);
        tolayer3(B, ackpkt);
    }

    /* deliver all in-order buffered pkts */
    while (B_received[B_base % SEQSPACE]) {
        int idx = B_base % SEQSPACE;
        tolayer5(B, B_buffer[idx].payload);
        B_received[idx] = false;
        B_base++;
    }
}

/******************************************************************************
 * The following functions need be completed only for bi-directional messages *
 *****************************************************************************/

/* Note that with simplex transfer from a-to-B, there is no B_output() */
void B_output(struct msg message)
{
}

/* called when B's timer goes off */
void B_timerinterrupt(void)
{
}
