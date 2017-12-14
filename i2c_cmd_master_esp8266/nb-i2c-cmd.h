// ********************************************************
// *  Nicebots Pluggie I2C Command Set                    *
// *  ================================                    *
// *  This file defines the inter-MCU language codes      *
// *  ..................................................  *
// *  Author: Gustavo Casanova  / Nicebots                *
// *  ..................................................  *
// *  Version: 0.2                                        *
// *  2017-11-25 gustavo.casanova@nicebots.com            *
// ********************************************************

// https://drive.google.com/open?id=1pu6hOd0GmKWU5qMEgDtN57TDMxKzHRv6

// Hardware Commands
#define STDPB1_1		0xE9				/* Command to Set ATtiny85 PB1 = 1 */
#define AKDPB1_1		0x16				/* Acknowledge PB1 = 1 Command */
#define STDPB1_0		0xE1				/* Command to Set ATtiny85 PB1 = 0 */
#define AKDPB1_0		0x1E				/* Acknowledge PB1 = 0 Command */
#define STANAPB3		0xFB				/* Command to Set ATtiny85 PB3 = PWMx */
#define ACKANPB3		0x04				/* Acknowledge PB3 = PWMx Command */
#define READADC2		0xDA				/* Command to Read ATtiny85 ADC2 */
#define ACKNADC2		0x25				/* Acknowledge Read ADC2 Command */

// General Commands
#define NOP				0x00				/* Command NOP */
#define UNKNOWNC		0xFF				/* No-Ack / UNKNOWNC */
#define RESETINY		0x80				/* Command to Reset ATtiny85 */
#define ACKRESTY		0x7F				/* Acknowledge Reset Command */
#define INITTINY		0x81				/* Command to Initialize ATtiny85 */
#define ACKINITY		0x7E				/* Acknowledge Initialize Command */
#define GET_INFO		0x82				/* Command to Read Generic Info */
#define ACK_GETI		0x7D				/* Acknowledge Read Info Command */
#define REL_ANDT		0x83				/* Command to Release Analog Data on Hold */
#define ACK_RELD		0x7C				/* Acknowledge Release Data Command */
#define FIXPOSIT		0x84				/* Fix Positive half-cycles for ADC Vrms calculations */
#define ACKFXPOS		0x7B				/* Acknowledge Fix Positive Command */
#define FIXNEGAT		0x85				/* Fix Negative half-cycles for ADC Vrms calculations */
#define ACKFXNEG		0x7A				/* Acknowledge Fix Negative Command */

// Transfer & Programming Commands
#define READBUFF		0xA1				/* Command to Read Buffer Data */
#define ACKRDBUF		0x5E				/* Acknowledge Read Data Command */
#define WRITBUFF		0xA2				/* Command to Write Data to Buffer */
#define ACKWTBUF		0x5D				/* Acknowledge Write Data Command */
