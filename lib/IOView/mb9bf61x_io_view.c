/**************************************************************************/
/*!
	@file			mb9bf61x_io_view.c
	@author         Nemui Trinomius (http://nemuisan.blog.bai.ne.jp)
    @version        V2.00
    @date           2012.06.15
	@brief          Debugging I/O Definitions For MB9BF618T Devices.
					Based On ChaN's SFR Definitions thanks!

    @section HISTORY
		2012.05.09	V1.00	Start Here.
		2012.06.15	V2.00	Fixed Register Definitions.

    @section LICENSE
		BSD License. See Copyright.txt
*/
/********************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "mcu.h"

/* Defines -------------------------------------------------------------------*/
/* Flash Interface */
#define FASZR   (*(volatile uint32_t*)0x40000000)
#define FRWTR   (*(volatile uint32_t*)0x40000004)
#define FSTR    (*(volatile uint32_t*)0x40000008)
#define FSYNDN  (*(volatile uint32_t*)0x40000010)
#define FBFCR   (*(volatile uint32_t*)0x40000014)
#define CRTRMM  (*(volatile uint16_t*)0x40000100)

/* Clock/Reset */
#define SCM_CTL    (*(volatile uint32_t*)0x40010000)
#define SCM_STR    (*(volatile uint32_t*)0x40010004)
#define STB_CTL    (*(volatile uint32_t*)0x40010008)
#define RST_STR    (*(volatile uint32_t*)0x4001000C)
#define BSC_PSR    (*(volatile uint32_t*)0x40010010)
#define APBC0_PSR  (*(volatile uint32_t*)0x40010014)
#define APBC1_PSR  (*(volatile uint32_t*)0x40010018)
#define APBC2_PSR  (*(volatile uint32_t*)0x4001001C)
#define SWC_PSR    (*(volatile uint32_t*)0x40010020)
#define TTC_PSR    (*(volatile uint32_t*)0x40010028)
#define CSW_TMR    (*(volatile uint32_t*)0x40010030)
#define PSW_TMR    (*(volatile uint32_t*)0x40010034)
#define PLL_CTL1   (*(volatile uint32_t*)0x40010038)
#define PLL_CTL2   (*(volatile uint32_t*)0x4001003C)
#define CSV_CTL    (*(volatile uint32_t*)0x40010040)
#define CSV_STR    (*(volatile uint32_t*)0x40010044)
#define FCSWH_CTL  (*(volatile uint32_t*)0x40010048)
#define FCSWL_CTL  (*(volatile uint32_t*)0x4001004C)
#define FCSWD_CTL  (*(volatile uint32_t*)0x40010050)
#define DBWDT_CTL  (*(volatile uint32_t*)0x40010054)
#define INT_ENR    (*(volatile uint32_t*)0x40010060)
#define INT_STR    (*(volatile uint32_t*)0x40010064)
#define INT_CLR    (*(volatile uint32_t*)0x40010068)

/* HW WDT */
#define WDG_LDR	(*(volatile uint32_t*)0x40011000)
#define WDG_VLR	(*(volatile uint32_t*)0x40011004)
#define WDG_CTL	(*(volatile uint32_t*)0x40011008)
#define WDG_ICL	(*(volatile uint32_t*)0x4001100C)
#define WDG_RIS	(*(volatile uint32_t*)0x40011010)
#define WDG_LCK	(*(volatile uint32_t*)0x40011C00)

/* SW WDT */
#define WdogLoad    (*(volatile uint32_t*)0x40012000)
#define WdogValue   (*(volatile uint32_t*)0x40012004)
#define WdogControl (*(volatile uint32_t*)0x40012008)
#define WdogIntClr  (*(volatile uint32_t*)0x4001200C)
#define WdogRIS     (*(volatile uint32_t*)0x40012010)
#define WdogLock    (*(volatile uint32_t*)0x40012C00)

/* Dual Timer */
#define DT_Timer1Load    (*(volatile uint32_t*)0x40015000)
#define DT_Timer1Value   (*(volatile uint32_t*)0x40015004)
#define DT_Timer1Control (*(volatile uint32_t*)0x40015008)
#define DT_Timer1IntClr  (*(volatile uint32_t*)0x4001500C)
#define DT_Timer1RIS     (*(volatile uint32_t*)0x40015010)
#define DT_Timer1MIS     (*(volatile uint32_t*)0x40015014)
#define DT_Timer1BGLoad  (*(volatile uint32_t*)0x40015018)
#define DT_Timer2Load    (*(volatile uint32_t*)0x40015020)
#define DT_Timer2Value   (*(volatile uint32_t*)0x40015024)
#define DT_Timer2Control (*(volatile uint32_t*)0x40015028)
#define DT_Timer2IntClr  (*(volatile uint32_t*)0x4001502C)
#define DT_Timer2RIS     (*(volatile uint32_t*)0x40015030)
#define DT_Timer2MIS     (*(volatile uint32_t*)0x40015034)
#define DT_Timer2BGLoad  (*(volatile uint32_t*)0x40015038)

/* MFT0/MFT1/MFT2 */
#define MFT0_OCCP0   (*(volatile uint16_t*)0x40020000)
#define MFT0_OCCP1   (*(volatile uint16_t*)0x40020004)
#define MFT0_OCCP2   (*(volatile uint16_t*)0x40020008)
#define MFT0_OCCP3   (*(volatile uint16_t*)0x4002000C)
#define MFT0_OCCP4   (*(volatile uint16_t*)0x40020010)
#define MFT0_OCCP5   (*(volatile uint16_t*)0x40020014)
#define MFT0_OCSA10  (*(volatile uint8_t *)0x40020018)
#define MFT0_OCSB10  (*(volatile uint8_t *)0x40020019)
#define MFT0_OCSA32  (*(volatile uint8_t *)0x4002001C)
#define MFT0_OCSB32  (*(volatile uint8_t *)0x4002001D)
#define MFT0_OCSA54  (*(volatile uint8_t *)0x40020020)
#define MFT0_OCSB54  (*(volatile uint8_t *)0x40020021)
#define MFT0_OCSC    (*(volatile uint8_t *)0x40020025)
#define MFT0_TCCP0   (*(volatile uint16_t*)0x40020028)
#define MFT0_TCDT0   (*(volatile uint16_t*)0x4002002C)
#define MFT0_TCSA0   (*(volatile uint16_t*)0x40020030)
#define MFT0_TCSB0   (*(volatile uint16_t*)0x40020034)
#define MFT0_TCCP1   (*(volatile uint16_t*)0x40020038)
#define MFT0_TCDT1   (*(volatile uint16_t*)0x4002003C)
#define MFT0_TCSA1   (*(volatile uint16_t*)0x40020040)
#define MFT0_TCSB1   (*(volatile uint16_t*)0x40020044)
#define MFT0_TCCP2   (*(volatile uint16_t*)0x40020048)
#define MFT0_TCDT2   (*(volatile uint16_t*)0x4002004C)
#define MFT0_TCSA2   (*(volatile uint16_t*)0x40020050)
#define MFT0_TCSB2   (*(volatile uint16_t*)0x40020054)
#define MFT0_OCFS10  (*(volatile uint8_t *)0x40020058)
#define MFT0_OCFS32  (*(volatile uint8_t *)0x40020059)
#define MFT0_OCFS54  (*(volatile uint8_t *)0x4002005C)
#define MFT0_ICFS10  (*(volatile uint8_t *)0x40020060)
#define MFT0_ICFS32  (*(volatile uint8_t *)0x40020061)
#define MFT0_ICCP0   (*(volatile uint16_t*)0x40020068)
#define MFT0_ICCP1   (*(volatile uint16_t*)0x4002006C)
#define MFT0_ICCP2   (*(volatile uint16_t*)0x40020070)
#define MFT0_ICCP3   (*(volatile uint16_t*)0x40020074)
#define MFT0_ICSA10  (*(volatile uint8_t *)0x40020078)
#define MFT0_ICSB10  (*(volatile uint8_t *)0x40020079)
#define MFT0_ICSA32  (*(volatile uint8_t *)0x4002007C)
#define MFT0_ICSB32  (*(volatile uint8_t *)0x4002007D)
#define MFT0_WFTM10  (*(volatile uint16_t*)0x40020080)
#define MFT0_WFTM32  (*(volatile uint16_t*)0x40020084)
#define MFT0_WFTM54  (*(volatile uint16_t*)0x40020088)
#define MFT0_WFSA10  (*(volatile uint16_t*)0x4002008C)
#define MFT0_WFSA32  (*(volatile uint16_t*)0x40020090)
#define MFT0_WFSA54  (*(volatile uint16_t*)0x40020094)
#define MFT0_WFIR    (*(volatile uint16_t*)0x40020098)
#define MFT0_NZCL    (*(volatile uint16_t*)0x4002009C)
#define MFT0_ACCP0   (*(volatile uint16_t*)0x400200A0)
#define MFT0_ACCPDN0 (*(volatile uint16_t*)0x400200A4)
#define MFT0_ACCP1   (*(volatile uint16_t*)0x400200A8)
#define MFT0_ACCPDN1 (*(volatile uint16_t*)0x400200AC)
#define MFT0_ACCP2   (*(volatile uint16_t*)0x400200B0)
#define MFT0_ACCPDN2 (*(volatile uint16_t*)0x400200B4)
#define MFT0_ACSB    (*(volatile uint8_t *)0x400200B8)
#define MFT0_ACSA    (*(volatile uint16_t*)0x400200BC)
#define MFT0_ATSA    (*(volatile uint16_t*)0x400200C0)

#define MFT1_OCCP0   (*(volatile uint16_t*)0x40021000)
#define MFT1_OCCP1   (*(volatile uint16_t*)0x40021004)
#define MFT1_OCCP2   (*(volatile uint16_t*)0x40021008)
#define MFT1_OCCP3   (*(volatile uint16_t*)0x4002100C)
#define MFT1_OCCP4   (*(volatile uint16_t*)0x40021010)
#define MFT1_OCCP5   (*(volatile uint16_t*)0x40021014)
#define MFT1_OCSA10  (*(volatile uint8_t *)0x40021018)
#define MFT1_OCSB10  (*(volatile uint8_t *)0x40021019)
#define MFT1_OCSA32  (*(volatile uint8_t *)0x4002101C)
#define MFT1_OCSB32  (*(volatile uint8_t *)0x4002101D)
#define MFT1_OCSA54  (*(volatile uint8_t *)0x40021020)
#define MFT1_OCSB54  (*(volatile uint8_t *)0x40021021)
#define MFT1_OCSC    (*(volatile uint8_t *)0x40021025)
#define MFT1_TCCP0   (*(volatile uint16_t*)0x40021028)
#define MFT1_TCDT0   (*(volatile uint16_t*)0x4002102C)
#define MFT1_TCSA0   (*(volatile uint16_t*)0x40021030)
#define MFT1_TCSB0   (*(volatile uint16_t*)0x40021034)
#define MFT1_TCCP1   (*(volatile uint16_t*)0x40021038)
#define MFT1_TCDT1   (*(volatile uint16_t*)0x4002103C)
#define MFT1_TCSA1   (*(volatile uint16_t*)0x40021040)
#define MFT1_TCSB1   (*(volatile uint16_t*)0x40021044)
#define MFT1_TCCP2   (*(volatile uint16_t*)0x40021048)
#define MFT1_TCDT2   (*(volatile uint16_t*)0x4002104C)
#define MFT1_TCSA2   (*(volatile uint16_t*)0x40021050)
#define MFT1_TCSB2   (*(volatile uint16_t*)0x40021054)
#define MFT1_OCFS10  (*(volatile uint8_t *)0x40021058)
#define MFT1_OCFS32  (*(volatile uint8_t *)0x40021059)
#define MFT1_OCFS54  (*(volatile uint8_t *)0x4002105C)
#define MFT1_ICFS10  (*(volatile uint8_t *)0x40021060)
#define MFT1_ICFS32  (*(volatile uint8_t *)0x40021061)
#define MFT1_ICCP0   (*(volatile uint16_t*)0x40021068)
#define MFT1_ICCP1   (*(volatile uint16_t*)0x4002106C)
#define MFT1_ICCP2   (*(volatile uint16_t*)0x40021070)
#define MFT1_ICCP3   (*(volatile uint16_t*)0x40021074)
#define MFT1_ICSA10  (*(volatile uint8_t *)0x40021078)
#define MFT1_ICSB10  (*(volatile uint8_t *)0x40021079)
#define MFT1_ICSA32  (*(volatile uint8_t *)0x4002107C)
#define MFT1_ICSB32  (*(volatile uint8_t *)0x4002107D)
#define MFT1_WFTM10  (*(volatile uint16_t*)0x40021080)
#define MFT1_WFTM32  (*(volatile uint16_t*)0x40021084)
#define MFT1_WFTM54  (*(volatile uint16_t*)0x40021088)
#define MFT1_WFSA10  (*(volatile uint16_t*)0x4002108C)
#define MFT1_WFSA32  (*(volatile uint16_t*)0x40021090)
#define MFT1_WFSA54  (*(volatile uint16_t*)0x40021094)
#define MFT1_WFIR    (*(volatile uint16_t*)0x40021098)
#define MFT1_NZCL    (*(volatile uint16_t*)0x4002109C)
#define MFT1_ACCP0   (*(volatile uint16_t*)0x400210A0)
#define MFT1_ACCPDN0 (*(volatile uint16_t*)0x400210A4)
#define MFT1_ACCP1   (*(volatile uint16_t*)0x400210A8)
#define MFT1_ACCPDN1 (*(volatile uint16_t*)0x400210AC)
#define MFT1_ACCP2   (*(volatile uint16_t*)0x400210B0)
#define MFT1_ACCPDN2 (*(volatile uint16_t*)0x400210B4)
#define MFT1_ACSB    (*(volatile uint8_t *)0x400210B8)
#define MFT1_ACSA    (*(volatile uint16_t*)0x400210BC)
#define MFT1_ATSA    (*(volatile uint16_t*)0x400210C0)

#define MFT2_OCCP0   (*(volatile uint16_t*)0x40022000)
#define MFT2_OCCP1   (*(volatile uint16_t*)0x40022004)
#define MFT2_OCCP2   (*(volatile uint16_t*)0x40022008)
#define MFT2_OCCP3   (*(volatile uint16_t*)0x4002200C)
#define MFT2_OCCP4   (*(volatile uint16_t*)0x40022010)
#define MFT2_OCCP5   (*(volatile uint16_t*)0x40022014)
#define MFT2_OCSA10  (*(volatile uint8_t *)0x40022018)
#define MFT2_OCSB10  (*(volatile uint8_t *)0x40022019)
#define MFT2_OCSA32  (*(volatile uint8_t *)0x4002201C)
#define MFT2_OCSB32  (*(volatile uint8_t *)0x4002201D)
#define MFT2_OCSA54  (*(volatile uint8_t *)0x40022020)
#define MFT2_OCSB54  (*(volatile uint8_t *)0x40022021)
#define MFT2_OCSC    (*(volatile uint8_t *)0x40022025)
#define MFT2_TCCP0   (*(volatile uint16_t*)0x40022028)
#define MFT2_TCDT0   (*(volatile uint16_t*)0x4002202C)
#define MFT2_TCSA0   (*(volatile uint16_t*)0x40022030)
#define MFT2_TCSB0   (*(volatile uint16_t*)0x40022034)
#define MFT2_TCCP1   (*(volatile uint16_t*)0x40022038)
#define MFT2_TCDT1   (*(volatile uint16_t*)0x4002203C)
#define MFT2_TCSA1   (*(volatile uint16_t*)0x40022040)
#define MFT2_TCSB1   (*(volatile uint16_t*)0x40022044)
#define MFT2_TCCP2   (*(volatile uint16_t*)0x40022048)
#define MFT2_TCDT2   (*(volatile uint16_t*)0x4002204C)
#define MFT2_TCSA2   (*(volatile uint16_t*)0x40022050)
#define MFT2_TCSB2   (*(volatile uint16_t*)0x40022054)
#define MFT2_OCFS10  (*(volatile uint8_t *)0x40022058)
#define MFT2_OCFS32  (*(volatile uint8_t *)0x40022059)
#define MFT2_OCFS54  (*(volatile uint8_t *)0x4002205C)
#define MFT2_ICFS10  (*(volatile uint8_t *)0x40022060)
#define MFT2_ICFS32  (*(volatile uint8_t *)0x40022061)
#define MFT2_ICCP0   (*(volatile uint16_t*)0x40022068)
#define MFT2_ICCP1   (*(volatile uint16_t*)0x4002206C)
#define MFT2_ICCP2   (*(volatile uint16_t*)0x40022070)
#define MFT2_ICCP3   (*(volatile uint16_t*)0x40022074)
#define MFT2_ICSA10  (*(volatile uint8_t *)0x40022078)
#define MFT2_ICSB10  (*(volatile uint8_t *)0x40022079)
#define MFT2_ICSA32  (*(volatile uint8_t *)0x4002207C)
#define MFT2_ICSB32  (*(volatile uint8_t *)0x4002207D)
#define MFT2_WFTM10  (*(volatile uint16_t*)0x40022080)
#define MFT2_WFTM32  (*(volatile uint16_t*)0x40022084)
#define MFT2_WFTM54  (*(volatile uint16_t*)0x40022088)
#define MFT2_WFSA10  (*(volatile uint16_t*)0x4002208C)
#define MFT2_WFSA32  (*(volatile uint16_t*)0x40022090)
#define MFT2_WFSA54  (*(volatile uint16_t*)0x40022094)
#define MFT2_WFIR    (*(volatile uint16_t*)0x40022098)
#define MFT2_NZCL    (*(volatile uint16_t*)0x4002209C)
#define MFT2_ACCP0   (*(volatile uint16_t*)0x400220A0)
#define MFT2_ACCPDN0 (*(volatile uint16_t*)0x400220A4)
#define MFT2_ACCP1   (*(volatile uint16_t*)0x400220A8)
#define MFT2_ACCPDN1 (*(volatile uint16_t*)0x400220AC)
#define MFT2_ACCP2   (*(volatile uint16_t*)0x400220B0)
#define MFT2_ACCPDN2 (*(volatile uint16_t*)0x400220B4)
#define MFT2_ACSB    (*(volatile uint8_t *)0x400220B8)
#define MFT2_ACSA    (*(volatile uint16_t*)0x400220BC)
#define MFT2_ATSA    (*(volatile uint16_t*)0x400220C0)

/* PPG */
#define PPG_TTCR0   (*(volatile uint8_t *)0x40024001)
#define PPG_COMP0   (*(volatile uint8_t *)0x40024009)
#define PPG_COMP2   (*(volatile uint8_t *)0x4002400C)
#define PPG_COMP4   (*(volatile uint8_t *)0x40024011)
#define PPG_COMP6   (*(volatile uint8_t *)0x40024014)
#define PPG_TTCR1   (*(volatile uint8_t *)0x40024021)
#define PPG_COMP1   (*(volatile uint8_t *)0x40024029)
#define PPG_COMP3   (*(volatile uint8_t *)0x4002402C)
#define PPG_COMP5   (*(volatile uint8_t *)0x40024031)
#define PPG_COMP7   (*(volatile uint8_t *)0x40024034)
#define PPG_TTCR2   (*(volatile uint8_t *)0x40024041)
#define PPG_COMP8   (*(volatile uint8_t *)0x40024049)
#define PPG_COMP10  (*(volatile uint8_t *)0x4002404C)
#define PPG_COMP12  (*(volatile uint8_t *)0x40024051)
#define PPG_COMP14  (*(volatile uint8_t *)0x40024054)
#define PPG_TRG     (*(volatile uint16_t*)0x40024100)
#define PPG_REVC    (*(volatile uint16_t*)0x40024104)
#define PPG_TRG1    (*(volatile uint16_t*)0x40024140)
#define PPG_REVC1   (*(volatile uint16_t*)0x40024144)
#define PPG_PPGC1   (*(volatile uint8_t *)0x40024200)
#define PPG_PPGC0   (*(volatile uint8_t *)0x40024201)
#define PPG_PPGC3   (*(volatile uint8_t *)0x40024204)
#define PPG_PPGC2   (*(volatile uint8_t *)0x40024205)
#define PPG_PRLL0   (*(volatile uint8_t *)0x40024208)
#define PPG_PRLH0   (*(volatile uint8_t *)0x40024209)
#define PPG_PRLL1   (*(volatile uint8_t *)0x4002420C)
#define PPG_PRLH1   (*(volatile uint8_t *)0x4002420D)
#define PPG_PRLL2   (*(volatile uint8_t *)0x40024210)
#define PPG_PRLH2   (*(volatile uint8_t *)0x40024211)
#define PPG_PRLL3   (*(volatile uint8_t *)0x40024214)
#define PPG_PRLH3   (*(volatile uint8_t *)0x40024215)
#define PPG_GATEC0  (*(volatile uint8_t *)0x40024218)
#define PPG_PPGC5   (*(volatile uint8_t *)0x40024240)
#define PPG_PPGC4   (*(volatile uint8_t *)0x40024241)
#define PPG_PPGC7   (*(volatile uint8_t *)0x40024244)
#define PPG_PPGC6   (*(volatile uint8_t *)0x40024245)
#define PPG_PRLL4   (*(volatile uint8_t *)0x40024248)
#define PPG_PRLH4   (*(volatile uint8_t *)0x40024249)
#define PPG_PRLL5   (*(volatile uint8_t *)0x4002424C)
#define PPG_PRLH5   (*(volatile uint8_t *)0x4002424D)
#define PPG_PRLL6   (*(volatile uint8_t *)0x40024250)
#define PPG_PRLH6   (*(volatile uint8_t *)0x40024251)
#define PPG_PRLL7   (*(volatile uint8_t *)0x40024254)
#define PPG_PRLH7   (*(volatile uint8_t *)0x40024255)
#define PPG_GATEC4  (*(volatile uint8_t *)0x40024258)
#define PPG_PPGC9   (*(volatile uint8_t *)0x40024280)
#define PPG_PPGC8   (*(volatile uint8_t *)0x40024281)
#define PPG_PPGC11  (*(volatile uint8_t *)0x40024284)
#define PPG_PPGC10  (*(volatile uint8_t *)0x40024285)
#define PPG_PRLL8   (*(volatile uint8_t *)0x40024288)
#define PPG_PRLH8   (*(volatile uint8_t *)0x40024289)
#define PPG_PRLL9   (*(volatile uint8_t *)0x4002428C)
#define PPG_PRLH9   (*(volatile uint8_t *)0x4002428D)
#define PPG_PRLL10  (*(volatile uint8_t *)0x40024290)
#define PPG_PRLH10  (*(volatile uint8_t *)0x40024291)
#define PPG_PRLL11  (*(volatile uint8_t *)0x40024294)
#define PPG_PRLH11  (*(volatile uint8_t *)0x40024295)
#define PPG_GATEC8  (*(volatile uint8_t *)0x40024298)
#define PPG_PPGC13  (*(volatile uint8_t *)0x400242C0)
#define PPG_PPGC12  (*(volatile uint8_t *)0x400242C1)
#define PPG_PPGC15  (*(volatile uint8_t *)0x400242C4)
#define PPG_PPGC14  (*(volatile uint8_t *)0x400242C5)
#define PPG_PRLL12  (*(volatile uint8_t *)0x400242C8)
#define PPG_PRLH12  (*(volatile uint8_t *)0x400242C9)
#define PPG_PRLL13  (*(volatile uint8_t *)0x400242CC)
#define PPG_PRLH13  (*(volatile uint8_t *)0x400242CD)
#define PPG_PRLL14  (*(volatile uint8_t *)0x400242D0)
#define PPG_PRLH14  (*(volatile uint8_t *)0x400242D1)
#define PPG_PRLL15  (*(volatile uint8_t *)0x400242D4)
#define PPG_PRLH15  (*(volatile uint8_t *)0x400242D5)
#define PPG_GATEC12 (*(volatile uint8_t *)0x400242D8)
#define PPG_PPGC17  (*(volatile uint8_t *)0x40024300)
#define PPG_PPGC16  (*(volatile uint8_t *)0x40024301)
#define PPG_PPGC19  (*(volatile uint8_t *)0x40024304)
#define PPG_PPGC18  (*(volatile uint8_t *)0x40024305)
#define PPG_PRLL16  (*(volatile uint8_t *)0x40024308)
#define PPG_PRLH16  (*(volatile uint8_t *)0x40024309)
#define PPG_PRLL18  (*(volatile uint8_t *)0x40024310)
#define PPG_PRLH17  (*(volatile uint8_t *)0x4002430D)
#define PPG_PRLH18  (*(volatile uint8_t *)0x40024311)
#define PPG_PRLL19  (*(volatile uint8_t *)0x40024314)
#define PPG_PRLH19  (*(volatile uint8_t *)0x40024315)
#define PPG_GATEC16 (*(volatile uint8_t *)0x40024318)
#define PPG_PPGC21  (*(volatile uint8_t *)0x40024340)
#define PPG_PPGC20  (*(volatile uint8_t *)0x40024341)
#define PPG_PPGC23  (*(volatile uint8_t *)0x40024344)
#define PPG_PPGC22  (*(volatile uint8_t *)0x40024345)
#define PPG_PRLL20  (*(volatile uint8_t *)0x40024348)
#define PPG_PPGC22  (*(volatile uint8_t *)0x40024345)
#define PPG_PRLL20  (*(volatile uint8_t *)0x40024348)
#define PPG_PRLH20  (*(volatile uint8_t *)0x40024349)
#define PPG_PRLL21  (*(volatile uint8_t *)0x4002434C)
#define PPG_PRLH21  (*(volatile uint8_t *)0x4002434D)
#define PPG_PRLL22  (*(volatile uint8_t *)0x40024350)
#define PPG_PRLH22  (*(volatile uint8_t *)0x40024351)
#define PPG_PRLL23  (*(volatile uint8_t *)0x40024354)
#define PPG_PRLH23  (*(volatile uint8_t *)0x40024355)
#define PPG_GATEC20 (*(volatile uint8_t *)0x40024358)

/* BaseTimer ch0-15 */
#define BT0_PCSR    (*(volatile uint16_t*)0x40025000)
#define BT0_PRLL    (*(volatile uint16_t*)0x40025000)
#define BT0_PDUT    (*(volatile uint16_t*)0x40025004)
#define BT0_PRLH    (*(volatile uint16_t*)0x40025004)
#define BT0_DTBF    (*(volatile uint16_t*)0x40025004)
#define BT0_TMR     (*(volatile uint16_t*)0x40025008)
#define BT0_TMCR    (*(volatile uint16_t*)0x4002500C)
#define BT0_STC     (*(volatile uint8_t *)0x40025010)
#define BT0_TMCR2   (*(volatile uint8_t *)0x40025011)

#define BT1_PCSR    (*(volatile uint16_t*)0x40025040)
#define BT1_PRLL    (*(volatile uint16_t*)0x40025040)
#define BT1_PDUT    (*(volatile uint16_t*)0x40025044)
#define BT1_PRLH    (*(volatile uint16_t*)0x40025044)
#define BT1_DTBF    (*(volatile uint16_t*)0x40025044)
#define BT1_TMR     (*(volatile uint16_t*)0x40025048)
#define BT1_TMCR    (*(volatile uint16_t*)0x4002504C)
#define BT1_STC     (*(volatile uint8_t *)0x40025050)
#define BT1_TMCR2   (*(volatile uint8_t *)0x40025051)

#define BT2_PCSR    (*(volatile uint16_t*)0x40025080)
#define BT2_PRLL    (*(volatile uint16_t*)0x40025080)
#define BT2_PDUT    (*(volatile uint16_t*)0x40025084)
#define BT2_PRLH    (*(volatile uint16_t*)0x40025084)
#define BT2_DTBF    (*(volatile uint16_t*)0x40025084)
#define BT2_TMR     (*(volatile uint16_t*)0x40025088)
#define BT2_TMCR    (*(volatile uint16_t*)0x4002508C)
#define BT2_STC     (*(volatile uint8_t *)0x40025090)
#define BT2_TMCR2   (*(volatile uint8_t *)0x40025091)

#define BT3_PCSR    (*(volatile uint16_t*)0x400250C0)
#define BT3_PRLL    (*(volatile uint16_t*)0x400250C0)
#define BT3_PDUT    (*(volatile uint16_t*)0x400250C4)
#define BT3_PRLH    (*(volatile uint16_t*)0x400250C4)
#define BT3_DTBF    (*(volatile uint16_t*)0x400250C4)
#define BT3_TMR     (*(volatile uint16_t*)0x400250C8)
#define BT3_TMCR    (*(volatile uint16_t*)0x400250CC)
#define BT3_STC     (*(volatile uint8_t *)0x400250D0)
#define BT3_TMCR2   (*(volatile uint8_t *)0x400250D1)

#define BT4_PCSR    (*(volatile uint16_t*)0x40025200)
#define BT4_PRLL    (*(volatile uint16_t*)0x40025200)
#define BT4_PDUT    (*(volatile uint16_t*)0x40025204)
#define BT4_PRLH    (*(volatile uint16_t*)0x40025204)
#define BT4_DTBF    (*(volatile uint16_t*)0x40025204)
#define BT4_TMR     (*(volatile uint16_t*)0x40025208)
#define BT4_TMCR    (*(volatile uint16_t*)0x4002520C)
#define BT4_STC     (*(volatile uint8_t *)0x40025210)
#define BT4_TMCR2   (*(volatile uint8_t *)0x40025211)

#define BT5_PCSR    (*(volatile uint16_t*)0x40025240)
#define BT5_PRLL    (*(volatile uint16_t*)0x40025240)
#define BT5_PDUT    (*(volatile uint16_t*)0x40025244)
#define BT5_PRLH    (*(volatile uint16_t*)0x40025244)
#define BT5_DTBF    (*(volatile uint16_t*)0x40025244)
#define BT5_TMR     (*(volatile uint16_t*)0x40025248)
#define BT5_TMCR    (*(volatile uint16_t*)0x4002524C)
#define BT5_STC     (*(volatile uint8_t *)0x40025250)
#define BT5_TMCR2   (*(volatile uint8_t *)0x40025251)

#define BT6_PCSR    (*(volatile uint16_t*)0x40025280)
#define BT6_PRLL    (*(volatile uint16_t*)0x40025280)
#define BT6_PDUT    (*(volatile uint16_t*)0x40025284)
#define BT6_PRLH    (*(volatile uint16_t*)0x40025284)
#define BT6_DTBF    (*(volatile uint16_t*)0x40025284)
#define BT6_TMR     (*(volatile uint16_t*)0x40025288)
#define BT6_TMCR    (*(volatile uint16_t*)0x4002528C)
#define BT6_STC     (*(volatile uint8_t *)0x40025290)
#define BT6_TMCR2   (*(volatile uint8_t *)0x40025291)

#define BT7_PCSR    (*(volatile uint16_t*)0x400252C0)
#define BT7_PRLL    (*(volatile uint16_t*)0x400252C0)
#define BT7_PDUT    (*(volatile uint16_t*)0x400252C4)
#define BT7_PRLH    (*(volatile uint16_t*)0x400252C4)
#define BT7_DTBF    (*(volatile uint16_t*)0x400252C4)
#define BT7_TMR     (*(volatile uint16_t*)0x400252C8)
#define BT7_TMCR    (*(volatile uint16_t*)0x400252CC)
#define BT7_STC     (*(volatile uint8_t *)0x400252D0)
#define BT7_TMCR2   (*(volatile uint8_t *)0x400252D1)

#define BT8_PCSR    (*(volatile uint16_t*)0x40025400)
#define BT8_PRLL    (*(volatile uint16_t*)0x40025400)
#define BT8_PDUT    (*(volatile uint16_t*)0x40025404)
#define BT8_PRLH    (*(volatile uint16_t*)0x40025404)
#define BT8_DTBF    (*(volatile uint16_t*)0x40025404)
#define BT8_TMR     (*(volatile uint16_t*)0x40025408)
#define BT8_TMCR    (*(volatile uint16_t*)0x4002540C)
#define BT8_STC     (*(volatile uint8_t *)0x40025410)
#define BT8_TMCR2   (*(volatile uint8_t *)0x40025411)

#define BT9_PCSR    (*(volatile uint16_t*)0x40025440)
#define BT9_PRLL    (*(volatile uint16_t*)0x40025440)
#define BT9_PDUT    (*(volatile uint16_t*)0x40025444)
#define BT9_PRLH    (*(volatile uint16_t*)0x40025444)
#define BT9_DTBF    (*(volatile uint16_t*)0x40025444)
#define BT9_TMR     (*(volatile uint16_t*)0x40025448)
#define BT9_TMCR    (*(volatile uint16_t*)0x4002544C)
#define BT9_STC     (*(volatile uint8_t *)0x40025450)
#define BT9_TMCR2   (*(volatile uint8_t *)0x40025451)

#define BT10_PCSR   (*(volatile uint16_t*)0x40025480)
#define BT10_PRLL   (*(volatile uint16_t*)0x40025480)
#define BT10_PDUT   (*(volatile uint16_t*)0x40025484)
#define BT10_PRLH   (*(volatile uint16_t*)0x40025484)
#define BT10_DTBF   (*(volatile uint16_t*)0x40025484)
#define BT10_TMR    (*(volatile uint16_t*)0x40025488)
#define BT10_TMCR   (*(volatile uint16_t*)0x4002548C)
#define BT10_STC    (*(volatile uint8_t *)0x40025490)
#define BT10_TMCR2  (*(volatile uint8_t *)0x40025491)

#define BT11_PCSR   (*(volatile uint16_t*)0x400254C0)
#define BT11_PRLL   (*(volatile uint16_t*)0x400254C0)
#define BT11_PDUT   (*(volatile uint16_t*)0x400254C4)
#define BT11_PRLH   (*(volatile uint16_t*)0x400254C4)
#define BT11_DTBF   (*(volatile uint16_t*)0x400254C4)
#define BT11_TMR    (*(volatile uint16_t*)0x400254C8)
#define BT11_TMCR   (*(volatile uint16_t*)0x400254CC)
#define BT11_STC    (*(volatile uint8_t *)0x400254D0)
#define BT11_TMCR2  (*(volatile uint8_t *)0x400254D1)

#define BT12_PCSR   (*(volatile uint16_t*)0x40025600)
#define BT12_PRLL   (*(volatile uint16_t*)0x40025600)
#define BT12_PDUT   (*(volatile uint16_t*)0x40025604)
#define BT12_PRLH   (*(volatile uint16_t*)0x40025604)
#define BT12_DTBF   (*(volatile uint16_t*)0x40025604)
#define BT12_TMR    (*(volatile uint16_t*)0x40025608)
#define BT12_TMCR   (*(volatile uint16_t*)0x4002560C)
#define BT12_STC    (*(volatile uint8_t *)0x40025610)
#define BT12_TMCR2  (*(volatile uint8_t *)0x40025611)

#define BT13_PCSR   (*(volatile uint16_t*)0x40025640)
#define BT13_PRLL   (*(volatile uint16_t*)0x40025640)
#define BT13_PDUT   (*(volatile uint16_t*)0x40025644)
#define BT13_PRLH   (*(volatile uint16_t*)0x40025644)
#define BT13_DTBF   (*(volatile uint16_t*)0x40025644)
#define BT13_TMR    (*(volatile uint16_t*)0x40025648)
#define BT13_TMCR   (*(volatile uint16_t*)0x4002564C)
#define BT13_STC    (*(volatile uint8_t *)0x40025650)
#define BT13_TMCR2  (*(volatile uint8_t *)0x40025651)

#define BT14_PCSR   (*(volatile uint16_t*)0x40025680)
#define BT14_PRLL   (*(volatile uint16_t*)0x40025680)
#define BT14_PDUT   (*(volatile uint16_t*)0x40025684)
#define BT14_PRLH   (*(volatile uint16_t*)0x40025684)
#define BT14_DTBF   (*(volatile uint16_t*)0x40025684)
#define BT14_TMR    (*(volatile uint16_t*)0x40025688)
#define BT14_TMCR   (*(volatile uint16_t*)0x4002568C)
#define BT14_STC    (*(volatile uint8_t *)0x40025690)
#define BT14_TMCR2  (*(volatile uint8_t *)0x40025691)

#define BT15_PCSR   (*(volatile uint16_t*)0x400256C0)
#define BT15_PRLL   (*(volatile uint16_t*)0x400256C0)
#define BT15_PDUT   (*(volatile uint16_t*)0x400256C4)
#define BT15_PRLH   (*(volatile uint16_t*)0x400256C4)
#define BT15_DTBF   (*(volatile uint16_t*)0x400256C4)
#define BT15_TMR    (*(volatile uint16_t*)0x400256C8)
#define BT15_TMCR   (*(volatile uint16_t*)0x400256CC)
#define BT15_STC    (*(volatile uint8_t *)0x400256D0)
#define BT15_TMCR2  (*(volatile uint8_t *)0x400256D1)

#define BTSEL0123	(*(volatile uint8_t *)0x40025101)
#define BTSEL4567	(*(volatile uint8_t *)0x40025301)
#define BTSEL89AB	(*(volatile uint8_t *)0x40025501)
#define BTSELCDEF	(*(volatile uint8_t *)0x40025701)

/* Software-based Simulation Startup */
#define BTSSSR	(*(volatile uint16_t*)0x40025FFC)

/* QPRC ch0-2 */
#define QPRC0_QPCR  (*(volatile uint16_t*)0x40026000)
#define QPRC0_QRCR  (*(volatile uint16_t*)0x40026004)
#define QPRC0_QPCCR (*(volatile uint16_t*)0x40026008)
#define QPRC0_QPRCR (*(volatile uint16_t*)0x4002600C)
#define QPRC0_QMPR  (*(volatile uint16_t*)0x40026010)
#define QPRC0_QICRL (*(volatile uint8_t *)0x40026014)
#define QPRC0_QICRH (*(volatile uint8_t *)0x40026015)
#define QPRC0_QCRL  (*(volatile uint8_t *)0x40026018)
#define QPRC0_QCRH  (*(volatile uint8_t *)0x40026019)
#define QPRC0_QECR  (*(volatile uint16_t*)0x4002601C)
#define QPRC0_QRCRR (*(volatile uint16_t*)0x4002603C)
#define QPRC0_QPCRR (*(volatile uint16_t*)0x4002603E)

#define QPRC1_QPCR  (*(volatile uint16_t*)0x40026040)
#define QPRC1_QRCR  (*(volatile uint16_t*)0x40026044)
#define QPRC1_QPCCR (*(volatile uint16_t*)0x40026048)
#define QPRC1_QPRCR (*(volatile uint16_t*)0x4002604C)
#define QPRC1_QMPR  (*(volatile uint16_t*)0x40026050)
#define QPRC1_QICRL (*(volatile uint8_t *)0x40026054)
#define QPRC1_QICRH (*(volatile uint8_t *)0x40026055)
#define QPRC1_QCRL  (*(volatile uint8_t *)0x40026058)
#define QPRC1_QCRH  (*(volatile uint8_t *)0x40026059)
#define QPRC1_QECR  (*(volatile uint16_t*)0x4002605C)
#define QPRC1_QRCRR (*(volatile uint16_t*)0x4002607C)
#define QPRC1_QPCRR (*(volatile uint16_t*)0x4002607E)

#define QPRC2_QPCR  (*(volatile uint16_t*)0x40026080)
#define QPRC2_QRCR  (*(volatile uint16_t*)0x40026084)
#define QPRC2_QPCCR (*(volatile uint16_t*)0x40026088)
#define QPRC2_QPRCR (*(volatile uint16_t*)0x4002608C)
#define QPRC2_QMPR  (*(volatile uint16_t*)0x40026090)
#define QPRC2_QICRL (*(volatile uint8_t *)0x40026094)
#define QPRC2_QICRH (*(volatile uint8_t *)0x40026095)
#define QPRC2_QCRL  (*(volatile uint8_t *)0x40026098)
#define QPRC2_QCRH  (*(volatile uint8_t *)0x40026099)
#define QPRC2_QECR  (*(volatile uint16_t*)0x4002609C)
#define QPRC2_QRCRR (*(volatile uint16_t*)0x400260BC)
#define QPRC2_QPCRR (*(volatile uint16_t*)0x400260BE)

/* 12bit ADC unit0-2 */
#define ADC0_ADSR	(*(volatile uint8_t *)0x40027000)
#define ADC0_ADCR	(*(volatile uint8_t *)0x40027001)
#define ADC0_SFNS	(*(volatile uint8_t *)0x40027008)
#define ADC0_SCCR	(*(volatile uint8_t *)0x40027009)
#define ADC0_SCFD	(*(volatile uint32_t*)0x4002700C)
#define ADC0_SCIS2	(*(volatile uint8_t *)0x40027010)
#define ADC0_SCIS3	(*(volatile uint8_t *)0x40027011)
#define ADC0_SCIS0	(*(volatile uint8_t *)0x40027014)
#define ADC0_SCIS1	(*(volatile uint8_t *)0x40027015)
#define ADC0_PFNS	(*(volatile uint8_t *)0x40027018)
#define ADC0_PCCR	(*(volatile uint8_t *)0x40027019)
#define ADC0_PCFD	(*(volatile uint32_t*)0x4002701C)
#define ADC0_PCIS	(*(volatile uint8_t *)0x40027020)
#define ADC0_CMPCR	(*(volatile uint8_t *)0x40027024)
#define ADC0_CMPD	(*(volatile uint16_t*)0x40027026)
#define ADC0_ADSS2	(*(volatile uint8_t *)0x40027028)
#define ADC0_ADSS3	(*(volatile uint8_t *)0x40027029)
#define ADC0_ADSS0	(*(volatile uint8_t *)0x4002702C)
#define ADC0_ADSS1	(*(volatile uint8_t *)0x4002702D)
#define ADC0_ADST1	(*(volatile uint8_t *)0x40027030)
#define ADC0_ADST0	(*(volatile uint8_t *)0x40027031)
#define ADC0_ADCT	(*(volatile uint8_t *)0x40027034)
#define ADC0_PRTSL	(*(volatile uint8_t *)0x40027038)
#define ADC0_SCTSL	(*(volatile uint8_t *)0x40027039)
#define ADC0_ADCEN	(*(volatile uint8_t *)0x4002703C)

#define ADC1_ADSR	(*(volatile uint8_t *)0x40027100)
#define ADC1_ADCR	(*(volatile uint8_t *)0x40027101)
#define ADC1_SFNS	(*(volatile uint8_t *)0x40027108)
#define ADC1_SCCR	(*(volatile uint8_t *)0x40027109)
#define ADC1_SCFD	(*(volatile uint32_t*)0x4002710C)
#define ADC1_SCIS2	(*(volatile uint8_t *)0x40027110)
#define ADC1_SCIS3	(*(volatile uint8_t *)0x40027111)
#define ADC1_SCIS0	(*(volatile uint8_t *)0x40027114)
#define ADC1_SCIS1	(*(volatile uint8_t *)0x40027115)
#define ADC1_PFNS	(*(volatile uint8_t *)0x40027118)
#define ADC1_PCCR	(*(volatile uint8_t *)0x40027119)
#define ADC1_PCFD	(*(volatile uint32_t*)0x4002711C)
#define ADC1_PCIS	(*(volatile uint8_t *)0x40027120)
#define ADC1_CMPCR	(*(volatile uint8_t *)0x40027124)
#define ADC1_CMPD	(*(volatile uint16_t*)0x40027126)
#define ADC1_ADSS2	(*(volatile uint8_t *)0x40027128)
#define ADC1_ADSS3	(*(volatile uint8_t *)0x40027129)
#define ADC1_ADSS0	(*(volatile uint8_t *)0x4002712C)
#define ADC1_ADSS1	(*(volatile uint8_t *)0x4002712D)
#define ADC1_ADST1	(*(volatile uint8_t *)0x40027130)
#define ADC1_ADST0	(*(volatile uint8_t *)0x40027131)
#define ADC1_ADCT	(*(volatile uint8_t *)0x40027134)
#define ADC1_PRTSL	(*(volatile uint8_t *)0x40027138)
#define ADC1_SCTSL	(*(volatile uint8_t *)0x40027139)
#define ADC1_ADCEN	(*(volatile uint8_t *)0x4002713C)

#define ADC2_ADSR	(*(volatile uint8_t *)0x40027200)
#define ADC2_ADCR	(*(volatile uint8_t *)0x40027201)
#define ADC2_SFNS	(*(volatile uint8_t *)0x40027208)
#define ADC2_SCCR	(*(volatile uint8_t *)0x40027209)
#define ADC2_SCFD	(*(volatile uint32_t*)0x4002720C)
#define ADC2_SCIS2	(*(volatile uint8_t *)0x40027210)
#define ADC2_SCIS3	(*(volatile uint8_t *)0x40027211)
#define ADC2_SCIS0	(*(volatile uint8_t *)0x40027214)
#define ADC2_SCIS1	(*(volatile uint8_t *)0x40027215)
#define ADC2_PFNS	(*(volatile uint8_t *)0x40027218)
#define ADC2_PCCR	(*(volatile uint8_t *)0x40027219)
#define ADC2_PCFD	(*(volatile uint32_t*)0x4002721C)
#define ADC2_PCIS	(*(volatile uint8_t *)0x40027220)
#define ADC2_CMPCR	(*(volatile uint8_t *)0x40027224)
#define ADC2_CMPD	(*(volatile uint16_t*)0x40027226)
#define ADC2_ADSS2	(*(volatile uint8_t *)0x40027228)
#define ADC2_ADSS3	(*(volatile uint8_t *)0x40027229)
#define ADC2_ADSS0	(*(volatile uint8_t *)0x4002722C)
#define ADC2_ADSS1	(*(volatile uint8_t *)0x4002722D)
#define ADC2_ADST1	(*(volatile uint8_t *)0x40027230)
#define ADC2_ADST0	(*(volatile uint8_t *)0x40027231)
#define ADC2_ADCT	(*(volatile uint8_t *)0x40027234)
#define ADC2_PRTSL	(*(volatile uint8_t *)0x40027238)
#define ADC2_SCTSL	(*(volatile uint8_t *)0x40027239)
#define ADC2_ADCEN	(*(volatile uint8_t *)0x4002723C)

/* CR Trim */
#define MCR_PSR   (*(volatile uint8_t *)0x4002E000)
#define MCR_FTRM  (*(volatile uint16_t*)0x4002E004)
#define MCR_RLR   (*(volatile uint32_t*)0x4002E00C)

/* EXTI */
#define EXTI_ENIR   (*(volatile uint32_t*)0x40030000)
#define EXTI_EIRR   (*(volatile uint32_t*)0x40030004)
#define EXTI_EICL   (*(volatile uint32_t*)0x40030008)
#define EXTI_ELVR   (*(volatile uint32_t*)0x4003000C)
#define EXTI_ELVR1  (*(volatile uint32_t*)0x40030010)
#define EXTI_NMIRR  (*(volatile uint8_t *)0x40030014)
#define EXTI_NMICL  (*(volatile uint8_t *)0x40030018)

/* INT-Req. Read */
#define DRQSEL    (*(volatile uint32_t*)0x40031000)
#define ODDPKS    (*(volatile uint8_t *)0x4003100B)
#define EXC02MON  (*(volatile uint32_t*)0x40031010)
#define IRQ00MON  (*(volatile uint32_t*)0x40031014)
#define IRQ01MON  (*(volatile uint32_t*)0x40031018)
#define IRQ02MON  (*(volatile uint32_t*)0x4003101C)
#define IRQ03MON  (*(volatile uint32_t*)0x40031020)
#define IRQ04MON  (*(volatile uint32_t*)0x40031024)
#define IRQ05MON  (*(volatile uint32_t*)0x40031028)
#define IRQ06MON  (*(volatile uint32_t*)0x4003102C)
#define IRQ07MON  (*(volatile uint32_t*)0x40031030)
#define IRQ08MON  (*(volatile uint32_t*)0x40031034)
#define IRQ09MON  (*(volatile uint32_t*)0x40031038)
#define IRQ10MON  (*(volatile uint32_t*)0x4003103C)
#define IRQ11MON  (*(volatile uint32_t*)0x40031040)
#define IRQ12MON  (*(volatile uint32_t*)0x40031044)
#define IRQ13MON  (*(volatile uint32_t*)0x40031048)
#define IRQ14MON  (*(volatile uint32_t*)0x4003104C)
#define IRQ15MON  (*(volatile uint32_t*)0x40031050)
#define IRQ16MON  (*(volatile uint32_t*)0x40031054)
#define IRQ17MON  (*(volatile uint32_t*)0x40031058)
#define IRQ18MON  (*(volatile uint32_t*)0x4003105C)
#define IRQ19MON  (*(volatile uint32_t*)0x40031060)
#define IRQ20MON  (*(volatile uint32_t*)0x40031064)
#define IRQ21MON  (*(volatile uint32_t*)0x40031068)
#define IRQ22MON  (*(volatile uint32_t*)0x4003106C)
#define IRQ23MON  (*(volatile uint32_t*)0x40031070)
#define IRQ24MON  (*(volatile uint32_t*)0x40031074)
#define IRQ25MON  (*(volatile uint32_t*)0x40031078)
#define IRQ26MON  (*(volatile uint32_t*)0x4003107C)
#define IRQ27MON  (*(volatile uint32_t*)0x40031080)
#define IRQ28MON  (*(volatile uint32_t*)0x40031084)
#define IRQ29MON  (*(volatile uint32_t*)0x40031088)
#define IRQ30MON  (*(volatile uint32_t*)0x4003108C)
#define IRQ31MON  (*(volatile uint32_t*)0x40031090)
#define IRQ32MON  (*(volatile uint32_t*)0x40031094)
#define IRQ33MON  (*(volatile uint32_t*)0x40031098)
#define IRQ34MON  (*(volatile uint32_t*)0x4003109C)
#define IRQ35MON  (*(volatile uint32_t*)0x400310A0)
#define IRQ36MON  (*(volatile uint32_t*)0x400310A4)
#define IRQ37MON  (*(volatile uint32_t*)0x400310A8)
#define IRQ38MON  (*(volatile uint32_t*)0x400310AC)
#define IRQ39MON  (*(volatile uint32_t*)0x400310B0)
#define IRQ40MON  (*(volatile uint32_t*)0x400310B4)
#define IRQ41MON  (*(volatile uint32_t*)0x400310B8)
#define IRQ42MON  (*(volatile uint32_t*)0x400310BC)
#define IRQ43MON  (*(volatile uint32_t*)0x400310C0)
#define IRQ44MON  (*(volatile uint32_t*)0x400310C4)
#define IRQ45MON  (*(volatile uint32_t*)0x400310C8)
#define IRQ46MON  (*(volatile uint32_t*)0x400310CC)
#define IRQ47MON  (*(volatile uint32_t*)0x400310D0)
#define DRQSEL1   (*(volatile uint32_t*)0x40031200)
#define DQESEL    (*(volatile uint32_t*)0x40031204)
#define ODDPKS1   (*(volatile uint8_t *)0x4003120F)

/* GPIO */
#define PFR0    (*(volatile uint32_t*)0x40033000)
#define PFR1    (*(volatile uint32_t*)0x40033004)
#define PFR2    (*(volatile uint32_t*)0x40033008)
#define PFR3    (*(volatile uint32_t*)0x4003300C)
#define PFR4    (*(volatile uint32_t*)0x40033010)
#define PFR5    (*(volatile uint32_t*)0x40033014)
#define PFR6    (*(volatile uint32_t*)0x40033018)
#define PFR7    (*(volatile uint32_t*)0x4003301C)
#define PFR8    (*(volatile uint32_t*)0x40033020)
#define PFR9    (*(volatile uint32_t*)0x40033024)
#define PFRA    (*(volatile uint32_t*)0x40033028)
#define PFRB    (*(volatile uint32_t*)0x4003302C)
#define PFRC    (*(volatile uint32_t*)0x40033030)
#define PFRD    (*(volatile uint32_t*)0x40033034)
#define PFRE    (*(volatile uint32_t*)0x40033038)
#define PFRF    (*(volatile uint32_t*)0x4003303C)
#define PCR0    (*(volatile uint32_t*)0x40033100)
#define PCR1    (*(volatile uint32_t*)0x40033104)
#define PCR2    (*(volatile uint32_t*)0x40033108)
#define PCR3    (*(volatile uint32_t*)0x4003310C)
#define PCR4    (*(volatile uint32_t*)0x40033110)
#define PCR5    (*(volatile uint32_t*)0x40033114)
#define PCR6    (*(volatile uint32_t*)0x40033118)
#define PCR7    (*(volatile uint32_t*)0x4003311C)
#define PCR8    (*(volatile uint32_t*)0x40033120)
#define PCR9    (*(volatile uint32_t*)0x40033124)
#define PCRA    (*(volatile uint32_t*)0x40033128)
#define PCRB    (*(volatile uint32_t*)0x4003312C)
#define PCRC    (*(volatile uint32_t*)0x40033130)
#define PCRD    (*(volatile uint32_t*)0x40033134)
#define PCRE    (*(volatile uint32_t*)0x40033138)
#define PCRF    (*(volatile uint32_t*)0x4003313C)
#define DDR0    (*(volatile uint32_t*)0x40033200)
#define DDR1    (*(volatile uint32_t*)0x40033204)
#define DDR2    (*(volatile uint32_t*)0x40033208)
#define DDR3    (*(volatile uint32_t*)0x4003320C)
#define DDR4    (*(volatile uint32_t*)0x40033210)
#define DDR5    (*(volatile uint32_t*)0x40033214)
#define DDR6    (*(volatile uint32_t*)0x40033218)
#define DDR7    (*(volatile uint32_t*)0x4003321C)
#define DDR8    (*(volatile uint32_t*)0x40033220)
#define DDR9    (*(volatile uint32_t*)0x40033224)
#define DDRA    (*(volatile uint32_t*)0x40033228)
#define DDRB    (*(volatile uint32_t*)0x4003322C)
#define DDRC    (*(volatile uint32_t*)0x40033230)
#define DDRD    (*(volatile uint32_t*)0x40033234)
#define DDRE    (*(volatile uint32_t*)0x40033238)
#define DDRF    (*(volatile uint32_t*)0x4003323C)
#define PDIR0   (*(volatile uint32_t*)0x40033300)
#define PDIR1   (*(volatile uint32_t*)0x40033304)
#define PDIR2   (*(volatile uint32_t*)0x40033308)
#define PDIR3   (*(volatile uint32_t*)0x4003330C)
#define PDIR4   (*(volatile uint32_t*)0x40033310)
#define PDIR5   (*(volatile uint32_t*)0x40033314)
#define PDIR6   (*(volatile uint32_t*)0x40033318)
#define PDIR7   (*(volatile uint32_t*)0x4003331C)
#define PDIR8   (*(volatile uint32_t*)0x40033320)
#define PDIR9   (*(volatile uint32_t*)0x40033324)
#define PDIRA   (*(volatile uint32_t*)0x40033328)
#define PDIRB   (*(volatile uint32_t*)0x4003332C)
#define PDIRC   (*(volatile uint32_t*)0x40033330)
#define PDIRD   (*(volatile uint32_t*)0x40033334)
#define PDIRE   (*(volatile uint32_t*)0x40033338)
#define PDIRF   (*(volatile uint32_t*)0x4003333C)
#define PDOR0   (*(volatile uint32_t*)0x40033400)
#define PDOR1   (*(volatile uint32_t*)0x40033404)
#define PDOR2   (*(volatile uint32_t*)0x40033408)
#define PDOR3   (*(volatile uint32_t*)0x4003340C)
#define PDOR4   (*(volatile uint32_t*)0x40033410)
#define PDOR5   (*(volatile uint32_t*)0x40033414)
#define PDOR6   (*(volatile uint32_t*)0x40033418)
#define PDOR7   (*(volatile uint32_t*)0x4003341C)
#define PDOR8   (*(volatile uint32_t*)0x40033420)
#define PDOR9   (*(volatile uint32_t*)0x40033424)
#define PDORA   (*(volatile uint32_t*)0x40033428)
#define PDORB   (*(volatile uint32_t*)0x4003342C)
#define PDORC   (*(volatile uint32_t*)0x40033430)
#define PDORD   (*(volatile uint32_t*)0x40033434)
#define PDORE   (*(volatile uint32_t*)0x40033438)
#define PDORF   (*(volatile uint32_t*)0x4003343C)
#define ADE     (*(volatile uint32_t*)0x40033500)
#define SPSR    (*(volatile uint32_t*)0x40033580)
#define EPFR00  (*(volatile uint32_t*)0x40033600)
#define EPFR01  (*(volatile uint32_t*)0x40033604)
#define EPFR02  (*(volatile uint32_t*)0x40033608)
#define EPFR03  (*(volatile uint32_t*)0x4003360C)
#define EPFR04  (*(volatile uint32_t*)0x40033610)
#define EPFR05  (*(volatile uint32_t*)0x40033614)
#define EPFR06  (*(volatile uint32_t*)0x40033618)
#define EPFR07  (*(volatile uint32_t*)0x4003361C)
#define EPFR08  (*(volatile uint32_t*)0x40033620)
#define EPFR09  (*(volatile uint32_t*)0x40033624)
#define EPFR10  (*(volatile uint32_t*)0x40033628)
#define EPFR11  (*(volatile uint32_t*)0x4003362C)
#define EPFR12  (*(volatile uint32_t*)0x40033630)
#define EPFR13  (*(volatile uint32_t*)0x40033634)
#define EPFR14  (*(volatile uint32_t*)0x40033638)
#define EPFR15  (*(volatile uint32_t*)0x4003363C)
#define PZR0    (*(volatile uint32_t*)0x40033700)
#define PZR1    (*(volatile uint32_t*)0x40033704)
#define PZR2    (*(volatile uint32_t*)0x40033708)
#define PZR3    (*(volatile uint32_t*)0x4003370C)
#define PZR4    (*(volatile uint32_t*)0x40033710)
#define PZR5    (*(volatile uint32_t*)0x40033714)
#define PZR6    (*(volatile uint32_t*)0x40033718)
#define PZR7    (*(volatile uint32_t*)0x4003371C)
#define PZR8    (*(volatile uint32_t*)0x40033720)
#define PZR9    (*(volatile uint32_t*)0x40033724)
#define PZRA    (*(volatile uint32_t*)0x40033728)
#define PZRB    (*(volatile uint32_t*)0x4003372C)
#define PZRC    (*(volatile uint32_t*)0x40033730)
#define PZRD    (*(volatile uint32_t*)0x40033734)
#define PZRE    (*(volatile uint32_t*)0x40033738)
#define PZRF    (*(volatile uint32_t*)0x4003373C)

/* LVD */
#define LVD_CTL  (*(volatile uint8_t *)0x40035000)
#define LVD_STR  (*(volatile uint8_t *)0x40035004)
#define LVD_CLR  (*(volatile uint8_t *)0x40035008)
#define LVD_RLR  (*(volatile uint32_t*)0x4003500C)
#define LVD_STR2 (*(volatile uint8_t *)0x40035010)

/* DS_Mode */
#define DSM_PMD_CTL (*(volatile uint8_t *)0x40035800)
#define DSM_WRFSR   (*(volatile uint8_t *)0x40035804)
#define DSM_WIFSR   (*(volatile uint8_t *)0x40035808)
#define DSM_WIER    (*(volatile uint8_t *)0x4003580C)
#define DSM_WILVR   (*(volatile uint8_t *)0x40035810)
#define DSM_BUR01   (*(volatile uint8_t *)0x40035900)
#define DSM_BUR02   (*(volatile uint8_t *)0x40035901)
#define DSM_BUR03   (*(volatile uint8_t *)0x40035902)
#define DSM_BUR04   (*(volatile uint8_t *)0x40035903)
#define DSM_BUR05   (*(volatile uint8_t *)0x40035904)
#define DSM_BUR06   (*(volatile uint8_t *)0x40035905)
#define DSM_BUR07   (*(volatile uint8_t *)0x40035906)
#define DSM_BUR08   (*(volatile uint8_t *)0x40035907)
#define DSM_BUR09   (*(volatile uint8_t *)0x40035908)
#define DSM_BUR10   (*(volatile uint8_t *)0x40035909)
#define DSM_BUR11   (*(volatile uint8_t *)0x4003590A)
#define DSM_BUR12   (*(volatile uint8_t *)0x4003590B)
#define DSM_BUR13   (*(volatile uint8_t *)0x4003590C)
#define DSM_BUR14   (*(volatile uint8_t *)0x4003590D)
#define DSM_BUR15   (*(volatile uint8_t *)0x4003590E)
#define DSM_BUR16   (*(volatile uint8_t *)0x4003590F)

/* USB Clock */
#define USBCLK_UCCR      (*(volatile uint8_t *)0x40036000)
#define USBCLK_UPCR1     (*(volatile uint8_t *)0x40036004)
#define USBCLK_UPCR2     (*(volatile uint8_t *)0x40036008)
#define USBCLK_UPCR3     (*(volatile uint8_t *)0x4003600C)
#define USBCLK_UPCR4     (*(volatile uint8_t *)0x40036010)
#define USBCLK_UP_STR    (*(volatile uint8_t *)0x40036014)
#define USBCLK_UPINT_ENR (*(volatile uint8_t *)0x40036018)
#define USBCLK_UPINT_CLR (*(volatile uint8_t *)0x4003601C)
#define USBCLK_UPINT_STR (*(volatile uint8_t *)0x40036020)
#define USBCLK_UPCR5     (*(volatile uint8_t *)0x40036024)
#define USBCLK_UPCR6     (*(volatile uint8_t *)0x40036028)
#define USBCLK_UPCR7     (*(volatile uint8_t *)0x4003602C)
#define USBCLK_USBEN     (*(volatile uint8_t *)0x40036030)
#define USBCLK_USBEN1    (*(volatile uint8_t *)0x40036034)

/* CAN Prescaler */
#define CANPRE  (*(volatile uint8_t *)0x40037000)

/* MFS ch0-3 */
#define MFS0_SMR   (*(volatile uint8_t *)0x40038000)
#define MFS0_SCR   (*(volatile uint8_t *)0x40038001)
#define MFS0_IBCR  (*(volatile uint8_t *)0x40038001)
#define MFS0_ESCR  (*(volatile uint8_t *)0x40038004)
#define MFS0_IBSR  (*(volatile uint8_t *)0x40038004)
#define MFS0_SSR   (*(volatile uint8_t *)0x40038005)
#define MFS0_TDR   (*(volatile uint16_t*)0x40038008)
#define MFS0_RDR   (*(volatile uint16_t*)0x40038008)
#define MFS0_BGR   (*(volatile uint16_t*)0x4003800C)
#define MFS0_ISBA  (*(volatile uint8_t *)0x40038010)
#define MFS0_ISMK  (*(volatile uint8_t *)0x40038011)

#define MFS1_SMR   (*(volatile uint8_t *)0x40038100)
#define MFS1_SCR   (*(volatile uint8_t *)0x40038101)
#define MFS1_IBCR  (*(volatile uint8_t *)0x40038101)
#define MFS1_ESCR  (*(volatile uint8_t *)0x40038104)
#define MFS1_IBSR  (*(volatile uint8_t *)0x40038104)
#define MFS1_SSR   (*(volatile uint8_t *)0x40038105)
#define MFS1_TDR   (*(volatile uint16_t*)0x40038108)
#define MFS1_RDR   (*(volatile uint16_t*)0x40038108)
#define MFS1_BGR   (*(volatile uint16_t*)0x4003810C)
#define MFS1_ISBA  (*(volatile uint8_t *)0x40038110)
#define MFS1_ISMK  (*(volatile uint8_t *)0x40038111)

#define MFS2_SMR   (*(volatile uint8_t *)0x40038200)
#define MFS2_SCR   (*(volatile uint8_t *)0x40038201)
#define MFS2_IBCR  (*(volatile uint8_t *)0x40038201)
#define MFS2_ESCR  (*(volatile uint8_t *)0x40038204)
#define MFS2_IBSR  (*(volatile uint8_t *)0x40038204)
#define MFS2_SSR   (*(volatile uint8_t *)0x40038205)
#define MFS2_TDR   (*(volatile uint16_t*)0x40038208)
#define MFS2_RDR   (*(volatile uint16_t*)0x40038208)
#define MFS2_BGR   (*(volatile uint16_t*)0x4003820C)
#define MFS2_ISBA  (*(volatile uint8_t *)0x40038210)
#define MFS2_ISMK  (*(volatile uint8_t *)0x40038211)

#define MFS3_SMR    (*(volatile uint8_t *)0x40038300)
#define MFS3_SCR    (*(volatile uint8_t *)0x40038301)
#define MFS3_IBCR   (*(volatile uint8_t *)0x40038301)
#define MFS3_ESCR   (*(volatile uint8_t *)0x40038304)
#define MFS3_IBSR   (*(volatile uint8_t *)0x40038304)
#define MFS3_SSR    (*(volatile uint8_t *)0x40038305)
#define MFS3_TDR    (*(volatile uint16_t*)0x40038308)
#define MFS3_RDR    (*(volatile uint16_t*)0x40038308)
#define MFS3_BGR    (*(volatile uint16_t*)0x4003830C)
#define MFS3_ISBA   (*(volatile uint8_t *)0x40038310)
#define MFS3_ISMK   (*(volatile uint8_t *)0x40038311)

/* MFS ch4-7 */
#define MFS4_SMR    (*(volatile uint8_t *)0x40038400)
#define MFS4_SCR    (*(volatile uint8_t *)0x40038401)
#define MFS4_IBCR   (*(volatile uint8_t *)0x40038401)
#define MFS4_ESCR   (*(volatile uint8_t *)0x40038404)
#define MFS4_IBSR   (*(volatile uint8_t *)0x40038404)
#define MFS4_SSR    (*(volatile uint8_t *)0x40038405)
#define MFS4_TDR    (*(volatile uint16_t*)0x40038408)
#define MFS4_RDR    (*(volatile uint16_t*)0x40038408)
#define MFS4_BGR    (*(volatile uint16_t*)0x4003840C)
#define MFS4_ISBA   (*(volatile uint8_t *)0x40038410)
#define MFS4_ISMK   (*(volatile uint8_t *)0x40038411)
#define MFS4_FCR    (*(volatile uint16_t*)0x40038414)
#define MFS4_FCR0   (*(volatile uint8_t *)0x40038414)
#define MFS4_FCR1   (*(volatile uint8_t *)0x40038415)
#define MFS4_FBYTE  (*(volatile uint16_t*)0x40038418)
#define MFS4_FBYTE1 (*(volatile uint8_t *)0x40038418)
#define MFS4_FBYTE2 (*(volatile uint8_t *)0x40038419)

#define MFS5_SMR    (*(volatile uint8_t *)0x40038500)
#define MFS5_SCR    (*(volatile uint8_t *)0x40038501)
#define MFS5_IBCR   (*(volatile uint8_t *)0x40038501)
#define MFS5_ESCR   (*(volatile uint8_t *)0x40038504)
#define MFS5_IBSR   (*(volatile uint8_t *)0x40038504)
#define MFS5_SSR    (*(volatile uint8_t *)0x40038505)
#define MFS5_TDR    (*(volatile uint16_t*)0x40038508)
#define MFS5_RDR    (*(volatile uint16_t*)0x40038508)
#define MFS5_BGR    (*(volatile uint16_t*)0x4003850C)
#define MFS5_ISBA   (*(volatile uint8_t *)0x40038510)
#define MFS5_ISMK   (*(volatile uint8_t *)0x40038511)
#define MFS5_FCR    (*(volatile uint16_t*)0x40038514)
#define MFS5_FCR0   (*(volatile uint8_t *)0x40038514)
#define MFS5_FCR1   (*(volatile uint8_t *)0x40038515)
#define MFS5_FBYTE  (*(volatile uint16_t*)0x40038518)
#define MFS5_FBYTE1 (*(volatile uint8_t *)0x40038518)
#define MFS5_FBYTE2 (*(volatile uint8_t *)0x40038519)

#define MFS6_SMR    (*(volatile uint8_t *)0x40038600)
#define MFS6_SCR    (*(volatile uint8_t *)0x40038601)
#define MFS6_IBCR   (*(volatile uint8_t *)0x40038601)
#define MFS6_ESCR   (*(volatile uint8_t *)0x40038604)
#define MFS6_IBSR   (*(volatile uint8_t *)0x40038604)
#define MFS6_SSR    (*(volatile uint8_t *)0x40038605)
#define MFS6_TDR    (*(volatile uint16_t*)0x40038608)
#define MFS6_RDR    (*(volatile uint16_t*)0x40038608)
#define MFS6_BGR    (*(volatile uint16_t*)0x4003860C)
#define MFS6_ISBA   (*(volatile uint8_t *)0x40038610)
#define MFS6_ISMK   (*(volatile uint8_t *)0x40038611)
#define MFS6_FCR    (*(volatile uint16_t*)0x40038614)
#define MFS6_FCR0   (*(volatile uint8_t *)0x40038614)
#define MFS6_FCR1   (*(volatile uint8_t *)0x40038615)
#define MFS6_FBYTE  (*(volatile uint16_t*)0x40038618)
#define MFS6_FBYTE1 (*(volatile uint8_t *)0x40038618)
#define MFS6_FBYTE2 (*(volatile uint8_t *)0x40038619)

#define MFS7_SMR    (*(volatile uint8_t *)0x40038700)
#define MFS7_SCR    (*(volatile uint8_t *)0x40038701)
#define MFS7_IBCR   (*(volatile uint8_t *)0x40038701)
#define MFS7_ESCR   (*(volatile uint8_t *)0x40038704)
#define MFS7_IBSR   (*(volatile uint8_t *)0x40038704)
#define MFS7_SSR    (*(volatile uint8_t *)0x40038705)
#define MFS7_TDR    (*(volatile uint16_t*)0x40038708)
#define MFS7_RDR    (*(volatile uint16_t*)0x40038708)
#define MFS7_BGR    (*(volatile uint16_t*)0x4003870C)
#define MFS7_ISBA   (*(volatile uint8_t *)0x40038710)
#define MFS7_ISMK   (*(volatile uint8_t *)0x40038711)
#define MFS7_FCR    (*(volatile uint16_t*)0x40038714)
#define MFS7_FCR0   (*(volatile uint8_t *)0x40038714)
#define MFS7_FCR1   (*(volatile uint8_t *)0x40038715)
#define MFS7_FBYTE  (*(volatile uint16_t*)0x40038718)
#define MFS7_FBYTE1 (*(volatile uint8_t *)0x40038718)
#define MFS7_FBYTE2 (*(volatile uint8_t *)0x40038719)

/* MSS Noise Filter */
#define MFS4_I2CDNF	(*(volatile uint32_t*)0x40038800)

/* CRC */
#define CRC_CRCCR   (*(volatile uint8_t *)0x40039000)
#define CRC_CRCINIT (*(volatile uint32_t*)0x40039004)
#define CRC_CRCIN   (*(volatile uint32_t*)0x40039008)
#define CRC_CRCR    (*(volatile uint32_t*)0x4003900C)

/* Watch Counter */
#define WC_WCRD     (*(volatile uint8_t *)0x4003A000)
#define WC_WCRL     (*(volatile uint8_t *)0x4003A001)
#define WC_WCCR     (*(volatile uint8_t *)0x4003A002)
#define WC_CLK_SEL  (*(volatile uint16_t*)0x4003A010)
#define WC_CLK_EN   (*(volatile uint8_t *)0x4003A014)

/* RTC */
#define RTC_WTCR1	(*(volatile uint32_t*)0x4003B000)
#define RTC_WTCR2	(*(volatile uint32_t*)0x4003B004)
#define RTC_WTBR	(*(volatile uint32_t*)0x4003B008)
#define RTC_WTSR	(*(volatile uint8_t *)0x4003B00C)
#define RTC_WTMIR	(*(volatile uint8_t *)0x4003B00D)
#define RTC_WTHR	(*(volatile uint8_t *)0x4003B00E)
#define RTC_WTDR	(*(volatile uint8_t *)0x4003B00F)
#define RTC_WTDW	(*(volatile uint8_t *)0x4003B010)
#define RTC_WTMOR	(*(volatile uint8_t *)0x4003B011)
#define RTC_WTYR	(*(volatile uint8_t *)0x4003B012)
#define RTC_ALMIR	(*(volatile uint8_t *)0x4003B015)
#define RTC_ALHR	(*(volatile uint8_t *)0x4003B016)
#define RTC_ALDR	(*(volatile uint8_t *)0x4003B017)
#define RTC_ALMOR	(*(volatile uint8_t *)0x4003B019)
#define RTC_ALYR	(*(volatile uint8_t *)0x4003B01A)
#define RTC_WTTR	(*(volatile uint32_t*)0x4003B01C)
#define RTC_WTCLKS	(*(volatile uint8_t *)0x4003B020)
#define RTC_WTCLKM	(*(volatile uint8_t *)0x4003B021)
#define RTC_WTCAL	(*(volatile uint8_t *)0x4003B024)
#define RTC_WTCALEN	(*(volatile uint8_t *)0x4003B025)
#define RTC_WTDIV	(*(volatile uint8_t *)0x4003B028)
#define RTC_WTDIVEN	(*(volatile uint8_t *)0x4003B029)

/* EXT-Bus I/F */
#define EXTBUS_MODE0  (*(volatile uint32_t*)0x4003F000)
#define EXTBUS_MODE1  (*(volatile uint32_t*)0x4003F004)
#define EXTBUS_MODE2  (*(volatile uint32_t*)0x4003F008)
#define EXTBUS_MODE3  (*(volatile uint32_t*)0x4003F00C)
#define EXTBUS_MODE4  (*(volatile uint32_t*)0x4003F010)
#define EXTBUS_MODE5  (*(volatile uint32_t*)0x4003F014)
#define EXTBUS_MODE6  (*(volatile uint32_t*)0x4003F018)
#define EXTBUS_MODE7  (*(volatile uint32_t*)0x4003F01C)
#define EXTBUS_TIM0   (*(volatile uint32_t*)0x4003F020)
#define EXTBUS_TIM1   (*(volatile uint32_t*)0x4003F024)
#define EXTBUS_TIM2   (*(volatile uint32_t*)0x4003F028)
#define EXTBUS_TIM3   (*(volatile uint32_t*)0x4003F02C)
#define EXTBUS_TIM4   (*(volatile uint32_t*)0x4003F030)
#define EXTBUS_TIM5   (*(volatile uint32_t*)0x4003F034)
#define EXTBUS_TIM6   (*(volatile uint32_t*)0x4003F038)
#define EXTBUS_TIM7   (*(volatile uint32_t*)0x4003F03C)
#define EXTBUS_AREA0  (*(volatile uint32_t*)0x4003F040)
#define EXTBUS_AREA1  (*(volatile uint32_t*)0x4003F044)
#define EXTBUS_AREA2  (*(volatile uint32_t*)0x4003F048)
#define EXTBUS_AREA3  (*(volatile uint32_t*)0x4003F04C)
#define EXTBUS_AREA4  (*(volatile uint32_t*)0x4003F050)
#define EXTBUS_AREA5  (*(volatile uint32_t*)0x4003F054)
#define EXTBUS_AREA6  (*(volatile uint32_t*)0x4003F058)
#define EXTBUS_AREA7  (*(volatile uint32_t*)0x4003F05C)
#define EXTBUS_ATIM0  (*(volatile uint32_t*)0x4003F060)
#define EXTBUS_ATIM1  (*(volatile uint32_t*)0x4003F064)
#define EXTBUS_ATIM2  (*(volatile uint32_t*)0x4003F068)
#define EXTBUS_ATIM3  (*(volatile uint32_t*)0x4003F06C)
#define EXTBUS_ATIM4  (*(volatile uint32_t*)0x4003F070)
#define EXTBUS_ATIM5  (*(volatile uint32_t*)0x4003F074)
#define EXTBUS_ATIM6  (*(volatile uint32_t*)0x4003F078)
#define EXTBUS_ATIM7  (*(volatile uint32_t*)0x4003F07C)
#define EXTBUS_DCLKR  (*(volatile uint32_t*)0x4003F300)

/* USB ch0-1 */
#define USB0_HCNT0     (*(volatile uint8_t *)0x40042100)
#define USB0_HCNT1     (*(volatile uint8_t *)0x40042101)
#define USB0_HIRQ      (*(volatile uint8_t *)0x40042104)
#define USB0_HERR      (*(volatile uint8_t *)0x40042105)
#define USB0_HSTATE    (*(volatile uint8_t *)0x40042108)
#define USB0_HFCOMP    (*(volatile uint8_t *)0x40042109)
#define USB0_HRTIMER10 (*(volatile uint16_t*)0x4004210C)
#define USB0_HRTIMER2  (*(volatile uint8_t *)0x40042110)
#define USB0_HADR      (*(volatile uint8_t *)0x40042111)
#define USB0_HEOF10    (*(volatile uint16_t*)0x40042114)
#define USB0_HFRAME10  (*(volatile uint16_t*)0x40042118)
#define USB0_HTOKEN    (*(volatile uint8_t *)0x4004211C)
#define USB0_UDCC      (*(volatile uint16_t*)0x40042120)
#define USB0_EP0C      (*(volatile uint16_t*)0x40042124)
#define USB0_EP1C      (*(volatile uint16_t*)0x40042128)
#define USB0_EP2C      (*(volatile uint16_t*)0x4004212C)
#define USB0_EP3C      (*(volatile uint16_t*)0x40042130)
#define USB0_EP4C      (*(volatile uint16_t*)0x40042134)
#define USB0_EP5C      (*(volatile uint16_t*)0x40042138)
#define USB0_TMSP      (*(volatile uint16_t*)0x4004213C)
#define USB0_UDCS      (*(volatile uint8_t *)0x40042140)
#define USB0_UDCIE     (*(volatile uint8_t *)0x40042141)
#define USB0_EP0IS     (*(volatile uint16_t*)0x40042144)
#define USB0_EP0OS     (*(volatile uint16_t*)0x40042148)
#define USB0_EP1S      (*(volatile uint16_t*)0x4004214C)
#define USB0_EP2S      (*(volatile uint16_t*)0x40042150)
#define USB0_EP3S      (*(volatile uint16_t*)0x40042154)
#define USB0_EP4S      (*(volatile uint16_t*)0x40042158)
#define USB0_EP5S      (*(volatile uint16_t*)0x4004215C)
#define USB0_EP0DTL    (*(volatile uint8_t *)0x40042160)
#define USB0_EP0DTH    (*(volatile uint8_t *)0x40042161)
#define USB0_EP1DTL    (*(volatile uint8_t *)0x40042164)
#define USB0_EP1DTH    (*(volatile uint8_t *)0x40042165)
#define USB0_EP2DTL    (*(volatile uint8_t *)0x40042168)
#define USB0_EP2DTH    (*(volatile uint8_t *)0x40042169)
#define USB0_EP3DTL    (*(volatile uint8_t *)0x4004216C)
#define USB0_EP3DTH    (*(volatile uint8_t *)0x4004216D)
#define USB0_EP4DTL    (*(volatile uint8_t *)0x40042170)
#define USB0_EP4DTH    (*(volatile uint8_t *)0x40042171)
#define USB0_EP5DTL    (*(volatile uint8_t *)0x40042174)
#define USB0_EP5DTH    (*(volatile uint8_t *)0x40042175)

#define USB1_HCNT0     (*(volatile uint8_t *)0x40052100)
#define USB1_HCNT1     (*(volatile uint8_t *)0x40052101)
#define USB1_HIRQ      (*(volatile uint8_t *)0x40052104)
#define USB1_HERR      (*(volatile uint8_t *)0x40052105)
#define USB1_HSTATE    (*(volatile uint8_t *)0x40052108)
#define USB1_HFCOMP    (*(volatile uint8_t *)0x40052109)
#define USB1_HRTIMER10 (*(volatile uint16_t*)0x4005210C)
#define USB1_HRTIMER2  (*(volatile uint8_t *)0x40052110)
#define USB1_HADR      (*(volatile uint8_t *)0x40052111)
#define USB1_HEOF10    (*(volatile uint16_t*)0x40052114)
#define USB1_HFRAME10  (*(volatile uint16_t*)0x40052118)
#define USB1_HTOKEN    (*(volatile uint8_t *)0x4005211C)
#define USB1_UDCC      (*(volatile uint16_t*)0x40052120)
#define USB1_EP0C      (*(volatile uint16_t*)0x40052124)
#define USB1_EP1C      (*(volatile uint16_t*)0x40052128)
#define USB1_EP2C      (*(volatile uint16_t*)0x4005212C)
#define USB1_EP3C      (*(volatile uint16_t*)0x40052130)
#define USB1_EP4C      (*(volatile uint16_t*)0x40052134)
#define USB1_EP5C      (*(volatile uint16_t*)0x40052138)
#define USB1_TMSP      (*(volatile uint16_t*)0x4005213C)
#define USB1_UDCS      (*(volatile uint8_t *)0x40052140)
#define USB1_UDCIE     (*(volatile uint8_t *)0x40052141)
#define USB1_EP0IS     (*(volatile uint16_t*)0x40052144)
#define USB1_EP0OS     (*(volatile uint16_t*)0x40052148)
#define USB1_EP1S      (*(volatile uint16_t*)0x4005214C)
#define USB1_EP2S      (*(volatile uint16_t*)0x40052150)
#define USB1_EP3S      (*(volatile uint16_t*)0x40052154)
#define USB1_EP4S      (*(volatile uint16_t*)0x40052158)
#define USB1_EP5S      (*(volatile uint16_t*)0x4005215C)
#define USB1_EP0DTL    (*(volatile uint8_t *)0x40052160)
#define USB1_EP0DTH    (*(volatile uint8_t *)0x40052161)
#define USB1_EP1DTL    (*(volatile uint8_t *)0x40052164)
#define USB1_EP1DTH    (*(volatile uint8_t *)0x40052165)
#define USB1_EP2DTL    (*(volatile uint8_t *)0x40052168)
#define USB1_EP2DTH    (*(volatile uint8_t *)0x40052169)
#define USB1_EP3DTL    (*(volatile uint8_t *)0x4005216C)
#define USB1_EP3DTH    (*(volatile uint8_t *)0x4005216D)
#define USB1_EP4DTL    (*(volatile uint8_t *)0x40052170)
#define USB1_EP4DTH    (*(volatile uint8_t *)0x40052171)
#define USB1_EP5DTL    (*(volatile uint8_t *)0x40052174)
#define USB1_EP5DTH    (*(volatile uint8_t *)0x40052175)

/* DMAC */
#define DMACR   (*(volatile uint32_t*)0x40060000)
#define DMACA0  (*(volatile uint32_t*)0x40060010)
#define DMACB0  (*(volatile uint32_t*)0x40060014)
#define DMACSA0 (*(volatile uint32_t*)0x40060018)
#define DMACDA0 (*(volatile uint32_t*)0x4006001C)
#define DMACA1  (*(volatile uint32_t*)0x40060020)
#define DMACB1  (*(volatile uint32_t*)0x40060024)
#define DMACSA1 (*(volatile uint32_t*)0x40060028)
#define DMACDA1 (*(volatile uint32_t*)0x4006002C)
#define DMACA2  (*(volatile uint32_t*)0x40060030)
#define DMACB2  (*(volatile uint32_t*)0x40060034)
#define DMACSA2 (*(volatile uint32_t*)0x40060038)
#define DMACDA2 (*(volatile uint32_t*)0x4006003C)
#define DMACA3  (*(volatile uint32_t*)0x40060040)
#define DMACB3  (*(volatile uint32_t*)0x40060044)
#define DMACSA3 (*(volatile uint32_t*)0x40060048)
#define DMACDA3 (*(volatile uint32_t*)0x4006004C)
#define DMACA4  (*(volatile uint32_t*)0x40060050)
#define DMACB4  (*(volatile uint32_t*)0x40060054)
#define DMACSA4 (*(volatile uint32_t*)0x40060058)
#define DMACDA4 (*(volatile uint32_t*)0x4006005C)
#define DMACA5  (*(volatile uint32_t*)0x40060060)
#define DMACB5  (*(volatile uint32_t*)0x40060064)
#define DMACSA5 (*(volatile uint32_t*)0x40060068)
#define DMACDA5 (*(volatile uint32_t*)0x4006006C)
#define DMACA6  (*(volatile uint32_t*)0x40060070)
#define DMACB6  (*(volatile uint32_t*)0x40060074)
#define DMACSA6 (*(volatile uint32_t*)0x40060078)
#define DMACDA6 (*(volatile uint32_t*)0x4006007C)
#define DMACA7  (*(volatile uint32_t*)0x40060080)
#define DMACB7  (*(volatile uint32_t*)0x40060084)
#define DMACSA7 (*(volatile uint32_t*)0x40060088)
#define DMACDA7 (*(volatile uint32_t*)0x4006008C)

/* CAN ch0-1 */
#define CAN0_CTRLR     (*(volatile uint16_t*)0x40062000)
#define CAN0_STATR     (*(volatile uint16_t*)0x40062002)
#define CAN0_ERRCNT    (*(volatile uint16_t*)0x40062004)
#define CAN0_BTR       (*(volatile uint16_t*)0x40062006)
#define CAN0_INTR      (*(volatile uint16_t*)0x40062008)
#define CAN0_TESTR     (*(volatile uint16_t*)0x4006200A)
#define CAN0_BRPER     (*(volatile uint16_t*)0x4006200C)
#define CAN0_IF1CREQ   (*(volatile uint16_t*)0x40062010)
#define CAN0_IF1CMSK   (*(volatile uint16_t*)0x40062010)
#define CAN0_IF1MSK1   (*(volatile uint16_t*)0x40062014)
#define CAN0_IF1MSK2   (*(volatile uint16_t*)0x40062016)
#define CAN0_IF1ARB1   (*(volatile uint16_t*)0x40062018)
#define CAN0_IF1ARB2   (*(volatile uint16_t*)0x4006201A)
#define CAN0_IF1MCTR   (*(volatile uint16_t*)0x4006201C)
#define CAN0_IF1DTA1LE (*(volatile uint16_t*)0x40062020)
#define CAN0_IF1DTA2LE (*(volatile uint16_t*)0x40062022)
#define CAN0_IF1DTB1LE (*(volatile uint16_t*)0x40062024)
#define CAN0_IF1DTB2LE (*(volatile uint16_t*)0x40062026)
#define CAN0_IF1DTA2BE (*(volatile uint16_t*)0x40062030)
#define CAN0_IF1DTA1BE (*(volatile uint16_t*)0x40062032)
#define CAN0_IF1DTB2BE (*(volatile uint16_t*)0x40062034)
#define CAN0_IF1DTB1BE (*(volatile uint16_t*)0x40062036)
#define CAN0_IF2CREQ   (*(volatile uint16_t*)0x40062040)
#define CAN0_IF2CMSK   (*(volatile uint16_t*)0x40062042)
#define CAN0_IF2MSK1   (*(volatile uint16_t*)0x40062044)
#define CAN0_IF2MSK2   (*(volatile uint16_t*)0x40062046)
#define CAN0_IF2ARB1   (*(volatile uint16_t*)0x40062048)
#define CAN0_IF2ARB2   (*(volatile uint16_t*)0x4006204A)
#define CAN0_IF2MCTR   (*(volatile uint16_t*)0x4006204C)
#define CAN0_IF2DTA1LE (*(volatile uint16_t*)0x40062050)
#define CAN0_IF2DTA2LE (*(volatile uint16_t*)0x40062052)
#define CAN0_IF2DTB1LE (*(volatile uint16_t*)0x40062054)
#define CAN0_IF2DTB2LE (*(volatile uint16_t*)0x40062056)
#define CAN0_IF2DTA2BE (*(volatile uint16_t*)0x40062060)
#define CAN0_IF2DTA1BE (*(volatile uint16_t*)0x40062062)
#define CAN0_IF2DTB2BE (*(volatile uint16_t*)0x40062064)
#define CAN0_IF2DTB1BE (*(volatile uint16_t*)0x40062066)

#define CAN1_CTRLR     (*(volatile uint16_t*)0x40063000)
#define CAN1_STATR     (*(volatile uint16_t*)0x40063002)
#define CAN1_ERRCNT    (*(volatile uint16_t*)0x40063004)
#define CAN1_BTR       (*(volatile uint16_t*)0x40063006)
#define CAN1_INTR      (*(volatile uint16_t*)0x40063008)
#define CAN1_TESTR     (*(volatile uint16_t*)0x4006300A)
#define CAN1_BRPER     (*(volatile uint16_t*)0x4006300C)
#define CAN1_IF1CREQ   (*(volatile uint16_t*)0x40063010)
#define CAN1_IF1CMSK   (*(volatile uint16_t*)0x40063010)
#define CAN1_IF1MSK1   (*(volatile uint16_t*)0x40063014)
#define CAN1_IF1MSK2   (*(volatile uint16_t*)0x40063016)
#define CAN1_IF1ARB1   (*(volatile uint16_t*)0x40063018)
#define CAN1_IF1ARB2   (*(volatile uint16_t*)0x4006301A)
#define CAN1_IF1MCTR   (*(volatile uint16_t*)0x4006301C)
#define CAN1_IF1DTA1LE (*(volatile uint16_t*)0x40063020)
#define CAN1_IF1DTA2LE (*(volatile uint16_t*)0x40063022)
#define CAN1_IF1DTB1LE (*(volatile uint16_t*)0x40063024)
#define CAN1_IF1DTB2LE (*(volatile uint16_t*)0x40063026)
#define CAN1_IF1DTA2BE (*(volatile uint16_t*)0x40063030)
#define CAN1_IF1DTA1BE (*(volatile uint16_t*)0x40063032)
#define CAN1_IF1DTB2BE (*(volatile uint16_t*)0x40063034)
#define CAN1_IF1DTB1BE (*(volatile uint16_t*)0x40063036)
#define CAN1_IF2CREQ   (*(volatile uint16_t*)0x40063040)
#define CAN1_IF2CMSK   (*(volatile uint16_t*)0x40063042)
#define CAN1_IF2MSK1   (*(volatile uint16_t*)0x40063044)
#define CAN1_IF2MSK2   (*(volatile uint16_t*)0x40063046)
#define CAN1_IF2ARB1   (*(volatile uint16_t*)0x40063048)
#define CAN1_IF2ARB2   (*(volatile uint16_t*)0x4006304A)
#define CAN1_IF2MCTR   (*(volatile uint16_t*)0x4006304C)
#define CAN1_IF2DTA1LE (*(volatile uint16_t*)0x40063050)
#define CAN1_IF2DTA2LE (*(volatile uint16_t*)0x40063052)
#define CAN1_IF2DTB1LE (*(volatile uint16_t*)0x40063054)
#define CAN1_IF2DTB2LE (*(volatile uint16_t*)0x40063056)
#define CAN1_IF2DTA2BE (*(volatile uint16_t*)0x40063060)
#define CAN1_IF2DTA1BE (*(volatile uint16_t*)0x40063062)
#define CAN1_IF2DTB2BE (*(volatile uint16_t*)0x40063064)
#define CAN1_IF2DTB1BE (*(volatile uint16_t*)0x40063066)



/* Cortex-M3 System timer */
#define	STCTRL		(*(volatile uint32_t*)0xE000E010)
#define	STRELOAD	(*(volatile uint32_t*)0xE000E014)
#define	STCURR		(*(volatile uint32_t*)0xE000E018)
#define	STCALIB		(*(volatile uint32_t*)0xE000E01C)

/* Cortex-M3 NVIC */
#define	ISER		( (volatile uint32_t*)0xE000E100)
#define	ICER		( (volatile uint32_t*)0xE000E180)
#define	ISPR		( (volatile uint32_t*)0xE000E200)
#define	ICPR		( (volatile uint32_t*)0xE000E280)
#define	IABR		( (volatile uint32_t*)0xE000E300)
#define	IPR			( (volatile uint8_t *)0xE000E400)
#define	STIR		(*(volatile uint32_t*)0xE000EF00)

/* Cortex-M3 SCB */
#define	ACTLR		(*(volatile uint32_t*)0xE000E008)
#define	CPUID		(*(volatile uint32_t*)0xE000ED00)
#define	ICSR		(*(volatile uint32_t*)0xE000ED04)
#define	VTOR		(*(volatile uint32_t*)0xE000ED08)
#define	AIRCR		(*(volatile uint32_t*)0xE000ED0C)
#define	SCR			(*(volatile uint32_t*)0xE000ED10)
#define	CCR			(*(volatile uint32_t*)0xE000ED14)
#define	SHPR		( (volatile uint8_t *)0xE000ED14)
#define	CFSR		(*(volatile uint32_t*)0xE000ED28)
#define	MMSR		(*(volatile uint32_t*)0xE000ED28)
#define	BFSR		(*(volatile uint32_t*)0xE000ED29)
#define	UFSR		(*(volatile uint32_t*)0xE000ED2A)
#define	HFSR		(*(volatile uint32_t*)0xE000ED2C)
#define	MMFAR		(*(volatile uint32_t*)0xE000ED34)
#define	BFAR		(*(volatile uint32_t*)0xE000ED38)

/* Constants -----------------------------------------------------------------*/
/* IO_Clock/Reset */
volatile uint32_t * const IO_SCM_CTL			__attribute__ ((section(".ioview"))) = &SCM_CTL;
volatile uint32_t * const IO_SCM_STR			__attribute__ ((section(".ioview"))) = &SCM_STR;
volatile uint32_t * const IO_STB_CTL			__attribute__ ((section(".ioview"))) = &STB_CTL;
volatile uint32_t * const IO_RST_STR			__attribute__ ((section(".ioview"))) = &RST_STR;
volatile uint32_t * const IO_BSC_PSR			__attribute__ ((section(".ioview"))) = &BSC_PSR;
volatile uint32_t * const IO_APBC0_PSR			__attribute__ ((section(".ioview"))) = &APBC0_PSR;
volatile uint32_t * const IO_APBC1_PSR 			__attribute__ ((section(".ioview"))) = &APBC1_PSR;
volatile uint32_t * const IO_APBC2_PSR 			__attribute__ ((section(".ioview"))) = &APBC2_PSR;
volatile uint32_t * const IO_SWC_PSR			__attribute__ ((section(".ioview"))) = &SWC_PSR;
volatile uint32_t * const IO_TTC_PSR			__attribute__ ((section(".ioview"))) = &TTC_PSR;
volatile uint32_t * const IO_CSW_TMR			__attribute__ ((section(".ioview"))) = &CSW_TMR;
volatile uint32_t * const IO_PSW_TMR			__attribute__ ((section(".ioview"))) = &PSW_TMR;
volatile uint32_t * const IO_PLL_CTL1			__attribute__ ((section(".ioview"))) = &PLL_CTL1;
volatile uint32_t * const IO_PLL_CTL2			__attribute__ ((section(".ioview"))) = &PLL_CTL2;
volatile uint32_t * const IO_CSV_CTL			__attribute__ ((section(".ioview"))) = &CSV_CTL;
volatile uint32_t * const IO_CSV_STR			__attribute__ ((section(".ioview"))) = &CSV_STR;
volatile uint32_t * const IO_FCSWH_CTL 			__attribute__ ((section(".ioview"))) = &FCSWH_CTL;
volatile uint32_t * const IO_FCSWL_CTL 			__attribute__ ((section(".ioview"))) = &FCSWL_CTL;
volatile uint32_t * const IO_FCSWD_CTL 			__attribute__ ((section(".ioview"))) = &FCSWD_CTL;
volatile uint32_t * const IO_DBWDT_CTL 			__attribute__ ((section(".ioview"))) = &DBWDT_CTL;
volatile uint32_t * const IO_INT_ENR			__attribute__ ((section(".ioview"))) = &INT_ENR;
volatile uint32_t * const IO_INT_STR			__attribute__ ((section(".ioview"))) = &INT_STR;
volatile uint32_t * const IO_INT_CLR			__attribute__ ((section(".ioview"))) = &INT_CLR;


/* IO_HW WDT */   
volatile uint32_t * const IO_WDG_LDR 			__attribute__ ((section(".ioview"))) = &WDG_LDR;
volatile uint32_t * const IO_WDG_VLR			__attribute__ ((section(".ioview"))) = &WDG_VLR;
volatile uint32_t * const IO_WDG_CTL			__attribute__ ((section(".ioview"))) = &WDG_CTL;
volatile uint32_t * const IO_WDG_ICL			__attribute__ ((section(".ioview"))) = &WDG_ICL;
volatile uint32_t * const IO_WDG_RIS			__attribute__ ((section(".ioview"))) = &WDG_RIS;
volatile uint32_t * const IO_WDG_LCK			__attribute__ ((section(".ioview"))) = &WDG_LCK;


/* IO_SW WDT */   
volatile uint32_t * const IO_WdogLoad			__attribute__ ((section(".ioview"))) = &WdogLoad;
volatile uint32_t * const IO_WdogValue			__attribute__ ((section(".ioview"))) = &WdogValue;
volatile uint32_t * const IO_WdogControl		__attribute__ ((section(".ioview"))) = &WdogControl;
volatile uint32_t * const IO_WdogIntClr 		__attribute__ ((section(".ioview"))) = &WdogIntClr;
volatile uint32_t * const IO_WdogRIS 			__attribute__ ((section(".ioview"))) = &WdogRIS;
volatile uint32_t * const IO_WdogLock			__attribute__ ((section(".ioview"))) = &WdogLock;


/* IO_Dual Timer */ 
volatile uint32_t * const IO_DT_Timer1Load		__attribute__ ((section(".ioview"))) = &DT_Timer1Load;
volatile uint32_t * const IO_DT_Timer1Value		__attribute__ ((section(".ioview"))) = &DT_Timer1Value;
volatile uint32_t * const IO_DT_Timer1Control	__attribute__ ((section(".ioview"))) = &DT_Timer1Control;
volatile uint32_t * const IO_DT_Timer1IntClr 	__attribute__ ((section(".ioview"))) = &DT_Timer1IntClr;
volatile uint32_t * const IO_DT_Timer1RIS 		__attribute__ ((section(".ioview"))) = &DT_Timer1RIS;
volatile uint32_t * const IO_DT_Timer1MIS 		__attribute__ ((section(".ioview"))) = &DT_Timer1MIS;
volatile uint32_t * const IO_DT_Timer1BGLoad 	__attribute__ ((section(".ioview"))) = &DT_Timer1BGLoad; 
volatile uint32_t * const IO_DT_Timer2Load		__attribute__ ((section(".ioview"))) = &DT_Timer2Load;
volatile uint32_t * const IO_DT_Timer2Value		__attribute__ ((section(".ioview"))) = &DT_Timer2Value;
volatile uint32_t * const IO_DT_Timer2Control	__attribute__ ((section(".ioview"))) = &DT_Timer2Control;
volatile uint32_t * const IO_DT_Timer2IntClr 	__attribute__ ((section(".ioview"))) = &DT_Timer2IntClr;
volatile uint32_t * const IO_DT_Timer2RIS 		__attribute__ ((section(".ioview"))) = &DT_Timer2RIS;
volatile uint32_t * const IO_DT_Timer2MIS 		__attribute__ ((section(".ioview"))) = &DT_Timer2MIS;
volatile uint32_t * const IO_DT_Timer2BGLoad 	__attribute__ ((section(".ioview"))) = &DT_Timer2BGLoad; 


/* IO_MFT0/MFT1/MFT2 */
volatile uint16_t * const IO_MFT0_OCCP0			__attribute__ ((section(".ioview"))) = &MFT0_OCCP0;
volatile uint16_t * const IO_MFT0_OCCP1			__attribute__ ((section(".ioview"))) = &MFT0_OCCP1;
volatile uint16_t * const IO_MFT0_OCCP2			__attribute__ ((section(".ioview"))) = &MFT0_OCCP2;
volatile uint16_t * const IO_MFT0_OCCP3			__attribute__ ((section(".ioview"))) = &MFT0_OCCP3;
volatile uint16_t * const IO_MFT0_OCCP4			__attribute__ ((section(".ioview"))) = &MFT0_OCCP4;
volatile uint16_t * const IO_MFT0_OCCP5			__attribute__ ((section(".ioview"))) = &MFT0_OCCP5;
volatile uint8_t  * const IO_MFT0_OCSA10 		__attribute__ ((section(".ioview"))) = &MFT0_OCSA10;
volatile uint8_t  * const IO_MFT0_OCSB10 		__attribute__ ((section(".ioview"))) = &MFT0_OCSB10;
volatile uint8_t  * const IO_MFT0_OCSA32 		__attribute__ ((section(".ioview"))) = &MFT0_OCSA32;
volatile uint8_t  * const IO_MFT0_OCSB32 		__attribute__ ((section(".ioview"))) = &MFT0_OCSB32;
volatile uint8_t  * const IO_MFT0_OCSA54 		__attribute__ ((section(".ioview"))) = &MFT0_OCSA54;
volatile uint8_t  * const IO_MFT0_OCSB54 		__attribute__ ((section(".ioview"))) = &MFT0_OCSB54;
volatile uint8_t  * const IO_MFT0_OCSC			__attribute__ ((section(".ioview"))) = &MFT0_OCSC;
volatile uint16_t * const IO_MFT0_TCCP0			__attribute__ ((section(".ioview"))) = &MFT0_TCCP0;
volatile uint16_t * const IO_MFT0_TCDT0			__attribute__ ((section(".ioview"))) = &MFT0_TCDT0;
volatile uint16_t * const IO_MFT0_TCSA0			__attribute__ ((section(".ioview"))) = &MFT0_TCSA0;
volatile uint16_t * const IO_MFT0_TCSB0			__attribute__ ((section(".ioview"))) = &MFT0_TCSB0;
volatile uint16_t * const IO_MFT0_TCCP1			__attribute__ ((section(".ioview"))) = &MFT0_TCCP1;
volatile uint16_t * const IO_MFT0_TCDT1			__attribute__ ((section(".ioview"))) = &MFT0_TCDT1;
volatile uint16_t * const IO_MFT0_TCSA1			__attribute__ ((section(".ioview"))) = &MFT0_TCSA1;
volatile uint16_t * const IO_MFT0_TCSB1			__attribute__ ((section(".ioview"))) = &MFT0_TCSB1;
volatile uint16_t * const IO_MFT0_TCCP2			__attribute__ ((section(".ioview"))) = &MFT0_TCCP2;
volatile uint16_t * const IO_MFT0_TCDT2			__attribute__ ((section(".ioview"))) = &MFT0_TCDT2;
volatile uint16_t * const IO_MFT0_TCSA2			__attribute__ ((section(".ioview"))) = &MFT0_TCSA2;
volatile uint16_t * const IO_MFT0_TCSB2			__attribute__ ((section(".ioview"))) = &MFT0_TCSB2;
volatile uint8_t  * const IO_MFT0_OCFS10 		__attribute__ ((section(".ioview"))) = &MFT0_OCFS10;
volatile uint8_t  * const IO_MFT0_OCFS32 		__attribute__ ((section(".ioview"))) = &MFT0_OCFS32;
volatile uint8_t  * const IO_MFT0_OCFS54 		__attribute__ ((section(".ioview"))) = &MFT0_OCFS54;
volatile uint8_t  * const IO_MFT0_ICFS10 		__attribute__ ((section(".ioview"))) = &MFT0_ICFS10;
volatile uint8_t  * const IO_MFT0_ICFS32 		__attribute__ ((section(".ioview"))) = &MFT0_ICFS32;
volatile uint16_t * const IO_MFT0_ICCP0			__attribute__ ((section(".ioview"))) = &MFT0_ICCP0;
volatile uint16_t * const IO_MFT0_ICCP1			__attribute__ ((section(".ioview"))) = &MFT0_ICCP1;
volatile uint16_t * const IO_MFT0_ICCP2			__attribute__ ((section(".ioview"))) = &MFT0_ICCP2;
volatile uint16_t * const IO_MFT0_ICCP3			__attribute__ ((section(".ioview"))) = &MFT0_ICCP3;
volatile uint8_t  * const IO_MFT0_ICSA10 		__attribute__ ((section(".ioview"))) = &MFT0_ICSA10;
volatile uint8_t  * const IO_MFT0_ICSB10 		__attribute__ ((section(".ioview"))) = &MFT0_ICSB10;
volatile uint8_t  * const IO_MFT0_ICSA32 		__attribute__ ((section(".ioview"))) = &MFT0_ICSA32;
volatile uint8_t  * const IO_MFT0_ICSB32 		__attribute__ ((section(".ioview"))) = &MFT0_ICSB32;
volatile uint16_t * const IO_MFT0_WFTM10 		__attribute__ ((section(".ioview"))) = &MFT0_WFTM10;
volatile uint16_t * const IO_MFT0_WFTM32 		__attribute__ ((section(".ioview"))) = &MFT0_WFTM32;
volatile uint16_t * const IO_MFT0_WFTM54 		__attribute__ ((section(".ioview"))) = &MFT0_WFTM54;
volatile uint16_t * const IO_MFT0_WFSA10 		__attribute__ ((section(".ioview"))) = &MFT0_WFSA10;
volatile uint16_t * const IO_MFT0_WFSA32 		__attribute__ ((section(".ioview"))) = &MFT0_WFSA32;
volatile uint16_t * const IO_MFT0_WFSA54 		__attribute__ ((section(".ioview"))) = &MFT0_WFSA54;
volatile uint16_t * const IO_MFT0_WFIR			__attribute__ ((section(".ioview"))) = &MFT0_WFIR;
volatile uint16_t * const IO_MFT0_NZCL			__attribute__ ((section(".ioview"))) = &MFT0_NZCL;
volatile uint16_t * const IO_MFT0_ACCP0			__attribute__ ((section(".ioview"))) = &MFT0_ACCP0;
volatile uint16_t * const IO_MFT0_ACCPDN0		__attribute__ ((section(".ioview"))) = &MFT0_ACCPDN0;
volatile uint16_t * const IO_MFT0_ACCP1			__attribute__ ((section(".ioview"))) = &MFT0_ACCP1;
volatile uint16_t * const IO_MFT0_ACCPDN1		__attribute__ ((section(".ioview"))) = &MFT0_ACCPDN1;
volatile uint16_t * const IO_MFT0_ACCP2			__attribute__ ((section(".ioview"))) = &MFT0_ACCP2;
volatile uint16_t * const IO_MFT0_ACCPDN2		__attribute__ ((section(".ioview"))) = &MFT0_ACCPDN2;
volatile uint8_t  * const IO_MFT0_ACSB			__attribute__ ((section(".ioview"))) = &MFT0_ACSB;
volatile uint16_t * const IO_MFT0_ACSA			__attribute__ ((section(".ioview"))) = &MFT0_ACSA;
volatile uint16_t * const IO_MFT0_ATSA			__attribute__ ((section(".ioview"))) = &MFT0_ATSA;

volatile uint16_t * const IO_MFT1_OCCP0			__attribute__ ((section(".ioview"))) = &MFT1_OCCP0;
volatile uint16_t * const IO_MFT1_OCCP1			__attribute__ ((section(".ioview"))) = &MFT1_OCCP1;
volatile uint16_t * const IO_MFT1_OCCP2			__attribute__ ((section(".ioview"))) = &MFT1_OCCP2;
volatile uint16_t * const IO_MFT1_OCCP3			__attribute__ ((section(".ioview"))) = &MFT1_OCCP3;
volatile uint16_t * const IO_MFT1_OCCP4			__attribute__ ((section(".ioview"))) = &MFT1_OCCP4;
volatile uint16_t * const IO_MFT1_OCCP5			__attribute__ ((section(".ioview"))) = &MFT1_OCCP5;
volatile uint8_t  * const IO_MFT1_OCSA10 		__attribute__ ((section(".ioview"))) = &MFT1_OCSA10;
volatile uint8_t  * const IO_MFT1_OCSB10 		__attribute__ ((section(".ioview"))) = &MFT1_OCSB10;
volatile uint8_t  * const IO_MFT1_OCSA32 		__attribute__ ((section(".ioview"))) = &MFT1_OCSA32;
volatile uint8_t  * const IO_MFT1_OCSB32 		__attribute__ ((section(".ioview"))) = &MFT1_OCSB32;
volatile uint8_t  * const IO_MFT1_OCSA54 		__attribute__ ((section(".ioview"))) = &MFT1_OCSA54;
volatile uint8_t  * const IO_MFT1_OCSB54 		__attribute__ ((section(".ioview"))) = &MFT1_OCSB54;
volatile uint8_t  * const IO_MFT1_OCSC			__attribute__ ((section(".ioview"))) = &MFT1_OCSC;
volatile uint16_t * const IO_MFT1_TCCP0			__attribute__ ((section(".ioview"))) = &MFT1_TCCP0;
volatile uint16_t * const IO_MFT1_TCDT0			__attribute__ ((section(".ioview"))) = &MFT1_TCDT0;
volatile uint16_t * const IO_MFT1_TCSA0			__attribute__ ((section(".ioview"))) = &MFT1_TCSA0;
volatile uint16_t * const IO_MFT1_TCSB0			__attribute__ ((section(".ioview"))) = &MFT1_TCSB0;
volatile uint16_t * const IO_MFT1_TCCP1			__attribute__ ((section(".ioview"))) = &MFT1_TCCP1;
volatile uint16_t * const IO_MFT1_TCDT1			__attribute__ ((section(".ioview"))) = &MFT1_TCDT1;
volatile uint16_t * const IO_MFT1_TCSA1			__attribute__ ((section(".ioview"))) = &MFT1_TCSA1;
volatile uint16_t * const IO_MFT1_TCSB1			__attribute__ ((section(".ioview"))) = &MFT1_TCSB1;
volatile uint16_t * const IO_MFT1_TCCP2			__attribute__ ((section(".ioview"))) = &MFT1_TCCP2;
volatile uint16_t * const IO_MFT1_TCDT2			__attribute__ ((section(".ioview"))) = &MFT1_TCDT2;
volatile uint16_t * const IO_MFT1_TCSA2			__attribute__ ((section(".ioview"))) = &MFT1_TCSA2;
volatile uint16_t * const IO_MFT1_TCSB2			__attribute__ ((section(".ioview"))) = &MFT1_TCSB2;
volatile uint8_t  * const IO_MFT1_OCFS10 		__attribute__ ((section(".ioview"))) = &MFT1_OCFS10;
volatile uint8_t  * const IO_MFT1_OCFS32 		__attribute__ ((section(".ioview"))) = &MFT1_OCFS32;
volatile uint8_t  * const IO_MFT1_OCFS54 		__attribute__ ((section(".ioview"))) = &MFT1_OCFS54;
volatile uint8_t  * const IO_MFT1_ICFS10 		__attribute__ ((section(".ioview"))) = &MFT1_ICFS10;
volatile uint8_t  * const IO_MFT1_ICFS32 		__attribute__ ((section(".ioview"))) = &MFT1_ICFS32;
volatile uint16_t * const IO_MFT1_ICCP0			__attribute__ ((section(".ioview"))) = &MFT1_ICCP0;
volatile uint16_t * const IO_MFT1_ICCP1			__attribute__ ((section(".ioview"))) = &MFT1_ICCP1;
volatile uint16_t * const IO_MFT1_ICCP2			__attribute__ ((section(".ioview"))) = &MFT1_ICCP2;
volatile uint16_t * const IO_MFT1_ICCP3			__attribute__ ((section(".ioview"))) = &MFT1_ICCP3;
volatile uint8_t  * const IO_MFT1_ICSA10 		__attribute__ ((section(".ioview"))) = &MFT1_ICSA10;
volatile uint8_t  * const IO_MFT1_ICSB10 		__attribute__ ((section(".ioview"))) = &MFT1_ICSB10;
volatile uint8_t  * const IO_MFT1_ICSA32 		__attribute__ ((section(".ioview"))) = &MFT1_ICSA32;
volatile uint8_t  * const IO_MFT1_ICSB32 		__attribute__ ((section(".ioview"))) = &MFT1_ICSB32;
volatile uint16_t * const IO_MFT1_WFTM10 		__attribute__ ((section(".ioview"))) = &MFT1_WFTM10;
volatile uint16_t * const IO_MFT1_WFTM32 		__attribute__ ((section(".ioview"))) = &MFT1_WFTM32;
volatile uint16_t * const IO_MFT1_WFTM54 		__attribute__ ((section(".ioview"))) = &MFT1_WFTM54;
volatile uint16_t * const IO_MFT1_WFSA10 		__attribute__ ((section(".ioview"))) = &MFT1_WFSA10;
volatile uint16_t * const IO_MFT1_WFSA32 		__attribute__ ((section(".ioview"))) = &MFT1_WFSA32;
volatile uint16_t * const IO_MFT1_WFSA54 		__attribute__ ((section(".ioview"))) = &MFT1_WFSA54;
volatile uint16_t * const IO_MFT1_WFIR			__attribute__ ((section(".ioview"))) = &MFT1_WFIR;
volatile uint16_t * const IO_MFT1_NZCL			__attribute__ ((section(".ioview"))) = &MFT1_NZCL;
volatile uint16_t * const IO_MFT1_ACCP0			__attribute__ ((section(".ioview"))) = &MFT1_ACCP0;
volatile uint16_t * const IO_MFT1_ACCPDN0		__attribute__ ((section(".ioview"))) = &MFT1_ACCPDN0;
volatile uint16_t * const IO_MFT1_ACCP1			__attribute__ ((section(".ioview"))) = &MFT1_ACCP1;
volatile uint16_t * const IO_MFT1_ACCPDN1		__attribute__ ((section(".ioview"))) = &MFT1_ACCPDN1;
volatile uint16_t * const IO_MFT1_ACCP2			__attribute__ ((section(".ioview"))) = &MFT1_ACCP2;
volatile uint16_t * const IO_MFT1_ACCPDN2		__attribute__ ((section(".ioview"))) = &MFT1_ACCPDN2;
volatile uint8_t  * const IO_MFT1_ACSB			__attribute__ ((section(".ioview"))) = &MFT1_ACSB;
volatile uint16_t * const IO_MFT1_ACSA			__attribute__ ((section(".ioview"))) = &MFT1_ACSA;
volatile uint16_t * const IO_MFT1_ATSA			__attribute__ ((section(".ioview"))) = &MFT1_ATSA;

volatile uint16_t * const IO_MFT2_OCCP0			__attribute__ ((section(".ioview"))) = &MFT2_OCCP0;
volatile uint16_t * const IO_MFT2_OCCP1			__attribute__ ((section(".ioview"))) = &MFT2_OCCP1;
volatile uint16_t * const IO_MFT2_OCCP2			__attribute__ ((section(".ioview"))) = &MFT2_OCCP2;
volatile uint16_t * const IO_MFT2_OCCP3			__attribute__ ((section(".ioview"))) = &MFT2_OCCP3;
volatile uint16_t * const IO_MFT2_OCCP4			__attribute__ ((section(".ioview"))) = &MFT2_OCCP4;
volatile uint16_t * const IO_MFT2_OCCP5			__attribute__ ((section(".ioview"))) = &MFT2_OCCP5;
volatile uint8_t  * const IO_MFT2_OCSA10 		__attribute__ ((section(".ioview"))) = &MFT2_OCSA10;
volatile uint8_t  * const IO_MFT2_OCSB10 		__attribute__ ((section(".ioview"))) = &MFT2_OCSB10;
volatile uint8_t  * const IO_MFT2_OCSA32 		__attribute__ ((section(".ioview"))) = &MFT2_OCSA32;
volatile uint8_t  * const IO_MFT2_OCSB32 		__attribute__ ((section(".ioview"))) = &MFT2_OCSB32;
volatile uint8_t  * const IO_MFT2_OCSA54 		__attribute__ ((section(".ioview"))) = &MFT2_OCSA54;
volatile uint8_t  * const IO_MFT2_OCSB54 		__attribute__ ((section(".ioview"))) = &MFT2_OCSB54;
volatile uint8_t  * const IO_MFT2_OCSC			__attribute__ ((section(".ioview"))) = &MFT2_OCSC;
volatile uint16_t * const IO_MFT2_TCCP0			__attribute__ ((section(".ioview"))) = &MFT2_TCCP0;
volatile uint16_t * const IO_MFT2_TCDT0			__attribute__ ((section(".ioview"))) = &MFT2_TCDT0;
volatile uint16_t * const IO_MFT2_TCSA0			__attribute__ ((section(".ioview"))) = &MFT2_TCSA0;
volatile uint16_t * const IO_MFT2_TCSB0			__attribute__ ((section(".ioview"))) = &MFT2_TCSB0;
volatile uint16_t * const IO_MFT2_TCCP1			__attribute__ ((section(".ioview"))) = &MFT2_TCCP1;
volatile uint16_t * const IO_MFT2_TCDT1			__attribute__ ((section(".ioview"))) = &MFT2_TCDT1;
volatile uint16_t * const IO_MFT2_TCSA1			__attribute__ ((section(".ioview"))) = &MFT2_TCSA1;
volatile uint16_t * const IO_MFT2_TCSB1			__attribute__ ((section(".ioview"))) = &MFT2_TCSB1;
volatile uint16_t * const IO_MFT2_TCCP2			__attribute__ ((section(".ioview"))) = &MFT2_TCCP2;
volatile uint16_t * const IO_MFT2_TCDT2			__attribute__ ((section(".ioview"))) = &MFT2_TCDT2;
volatile uint16_t * const IO_MFT2_TCSA2			__attribute__ ((section(".ioview"))) = &MFT2_TCSA2;
volatile uint16_t * const IO_MFT2_TCSB2			__attribute__ ((section(".ioview"))) = &MFT2_TCSB2;
volatile uint8_t  * const IO_MFT2_OCFS10 		__attribute__ ((section(".ioview"))) = &MFT2_OCFS10;
volatile uint8_t  * const IO_MFT2_OCFS32 		__attribute__ ((section(".ioview"))) = &MFT2_OCFS32;
volatile uint8_t  * const IO_MFT2_OCFS54 		__attribute__ ((section(".ioview"))) = &MFT2_OCFS54;
volatile uint8_t  * const IO_MFT2_ICFS10 		__attribute__ ((section(".ioview"))) = &MFT2_ICFS10;
volatile uint8_t  * const IO_MFT2_ICFS32 		__attribute__ ((section(".ioview"))) = &MFT2_ICFS32;
volatile uint16_t * const IO_MFT2_ICCP0			__attribute__ ((section(".ioview"))) = &MFT2_ICCP0;
volatile uint16_t * const IO_MFT2_ICCP1			__attribute__ ((section(".ioview"))) = &MFT2_ICCP1;
volatile uint16_t * const IO_MFT2_ICCP2			__attribute__ ((section(".ioview"))) = &MFT2_ICCP2;
volatile uint16_t * const IO_MFT2_ICCP3			__attribute__ ((section(".ioview"))) = &MFT2_ICCP3;
volatile uint8_t  * const IO_MFT2_ICSA10 		__attribute__ ((section(".ioview"))) = &MFT2_ICSA10;
volatile uint8_t  * const IO_MFT2_ICSB10 		__attribute__ ((section(".ioview"))) = &MFT2_ICSB10;
volatile uint8_t  * const IO_MFT2_ICSA32 		__attribute__ ((section(".ioview"))) = &MFT2_ICSA32;
volatile uint8_t  * const IO_MFT2_ICSB32 		__attribute__ ((section(".ioview"))) = &MFT2_ICSB32;
volatile uint16_t * const IO_MFT2_WFTM10 		__attribute__ ((section(".ioview"))) = &MFT2_WFTM10;
volatile uint16_t * const IO_MFT2_WFTM32 		__attribute__ ((section(".ioview"))) = &MFT2_WFTM32;
volatile uint16_t * const IO_MFT2_WFTM54 		__attribute__ ((section(".ioview"))) = &MFT2_WFTM54;
volatile uint16_t * const IO_MFT2_WFSA10 		__attribute__ ((section(".ioview"))) = &MFT2_WFSA10;
volatile uint16_t * const IO_MFT2_WFSA32 		__attribute__ ((section(".ioview"))) = &MFT2_WFSA32;
volatile uint16_t * const IO_MFT2_WFSA54 		__attribute__ ((section(".ioview"))) = &MFT2_WFSA54;
volatile uint16_t * const IO_MFT2_WFIR			__attribute__ ((section(".ioview"))) = &MFT2_WFIR;
volatile uint16_t * const IO_MFT2_NZCL			__attribute__ ((section(".ioview"))) = &MFT2_NZCL;
volatile uint16_t * const IO_MFT2_ACCP0			__attribute__ ((section(".ioview"))) = &MFT2_ACCP0;
volatile uint16_t * const IO_MFT2_ACCPDN0		__attribute__ ((section(".ioview"))) = &MFT2_ACCPDN0;
volatile uint16_t * const IO_MFT2_ACCP1			__attribute__ ((section(".ioview"))) = &MFT2_ACCP1;
volatile uint16_t * const IO_MFT2_ACCPDN1		__attribute__ ((section(".ioview"))) = &MFT2_ACCPDN1;
volatile uint16_t * const IO_MFT2_ACCP2			__attribute__ ((section(".ioview"))) = &MFT2_ACCP2;
volatile uint16_t * const IO_MFT2_ACCPDN2		__attribute__ ((section(".ioview"))) = &MFT2_ACCPDN2;
volatile uint8_t  * const IO_MFT2_ACSB			__attribute__ ((section(".ioview"))) = &MFT2_ACSB;
volatile uint16_t * const IO_MFT2_ACSA			__attribute__ ((section(".ioview"))) = &MFT2_ACSA;
volatile uint16_t * const IO_MFT2_ATSA			__attribute__ ((section(".ioview"))) = &MFT2_ATSA;

/* IO_PPG */
volatile uint8_t  * const IO_PPG_TTCR0			__attribute__ ((section(".ioview"))) = &PPG_TTCR0;
volatile uint8_t  * const IO_PPG_COMP0			__attribute__ ((section(".ioview"))) = &PPG_COMP0;
volatile uint8_t  * const IO_PPG_COMP2			__attribute__ ((section(".ioview"))) = &PPG_COMP2;
volatile uint8_t  * const IO_PPG_COMP4			__attribute__ ((section(".ioview"))) = &PPG_COMP4;
volatile uint8_t  * const IO_PPG_COMP6			__attribute__ ((section(".ioview"))) = &PPG_COMP6;
volatile uint8_t  * const IO_PPG_TTCR1			__attribute__ ((section(".ioview"))) = &PPG_TTCR1;
volatile uint8_t  * const IO_PPG_COMP1			__attribute__ ((section(".ioview"))) = &PPG_COMP1;
volatile uint8_t  * const IO_PPG_COMP3			__attribute__ ((section(".ioview"))) = &PPG_COMP3;
volatile uint8_t  * const IO_PPG_COMP5			__attribute__ ((section(".ioview"))) = &PPG_COMP5;
volatile uint8_t  * const IO_PPG_COMP7			__attribute__ ((section(".ioview"))) = &PPG_COMP7;
volatile uint8_t  * const IO_PPG_TTCR2			__attribute__ ((section(".ioview"))) = &PPG_TTCR2;
volatile uint8_t  * const IO_PPG_COMP8			__attribute__ ((section(".ioview"))) = &PPG_COMP8;
volatile uint8_t  * const IO_PPG_COMP10 		__attribute__ ((section(".ioview"))) = &PPG_COMP10;
volatile uint8_t  * const IO_PPG_COMP12 		__attribute__ ((section(".ioview"))) = &PPG_COMP12;
volatile uint8_t  * const IO_PPG_COMP14 		__attribute__ ((section(".ioview"))) = &PPG_COMP14;
volatile uint16_t * const IO_PPG_TRG 			__attribute__ ((section(".ioview"))) = &PPG_TRG;
volatile uint16_t * const IO_PPG_REVC			__attribute__ ((section(".ioview"))) = &PPG_REVC;
volatile uint16_t * const IO_PPG_TRG1			__attribute__ ((section(".ioview"))) = &PPG_TRG1;
volatile uint16_t * const IO_PPG_REVC1			__attribute__ ((section(".ioview"))) = &PPG_REVC1;
volatile uint8_t  * const IO_PPG_PPGC1			__attribute__ ((section(".ioview"))) = &PPG_PPGC1;
volatile uint8_t  * const IO_PPG_PPGC0			__attribute__ ((section(".ioview"))) = &PPG_PPGC0;
volatile uint8_t  * const IO_PPG_PPGC3			__attribute__ ((section(".ioview"))) = &PPG_PPGC3;
volatile uint8_t  * const IO_PPG_PPGC2			__attribute__ ((section(".ioview"))) = &PPG_PPGC2;
volatile uint8_t  * const IO_PPG_PRLL0			__attribute__ ((section(".ioview"))) = &PPG_PRLL0;
volatile uint8_t  * const IO_PPG_PRLH0			__attribute__ ((section(".ioview"))) = &PPG_PRLH0;
volatile uint8_t  * const IO_PPG_PRLL1			__attribute__ ((section(".ioview"))) = &PPG_PRLL1;
volatile uint8_t  * const IO_PPG_PRLH1			__attribute__ ((section(".ioview"))) = &PPG_PRLH1;
volatile uint8_t  * const IO_PPG_PRLL2			__attribute__ ((section(".ioview"))) = &PPG_PRLL2;
volatile uint8_t  * const IO_PPG_PRLH2			__attribute__ ((section(".ioview"))) = &PPG_PRLH2;
volatile uint8_t  * const IO_PPG_PRLL3			__attribute__ ((section(".ioview"))) = &PPG_PRLL3;
volatile uint8_t  * const IO_PPG_PRLH3			__attribute__ ((section(".ioview"))) = &PPG_PRLH3;
volatile uint8_t  * const IO_PPG_GATEC0 		__attribute__ ((section(".ioview"))) = &PPG_GATEC0;
volatile uint8_t  * const IO_PPG_PPGC5			__attribute__ ((section(".ioview"))) = &PPG_PPGC5;
volatile uint8_t  * const IO_PPG_PPGC4			__attribute__ ((section(".ioview"))) = &PPG_PPGC4;
volatile uint8_t  * const IO_PPG_PPGC7			__attribute__ ((section(".ioview"))) = &PPG_PPGC7;
volatile uint8_t  * const IO_PPG_PPGC6			__attribute__ ((section(".ioview"))) = &PPG_PPGC6;
volatile uint8_t  * const IO_PPG_PRLL4			__attribute__ ((section(".ioview"))) = &PPG_PRLL4;
volatile uint8_t  * const IO_PPG_PRLH4			__attribute__ ((section(".ioview"))) = &PPG_PRLH4;
volatile uint8_t  * const IO_PPG_PRLL5			__attribute__ ((section(".ioview"))) = &PPG_PRLL5;
volatile uint8_t  * const IO_PPG_PRLH5			__attribute__ ((section(".ioview"))) = &PPG_PRLH5;
volatile uint8_t  * const IO_PPG_PRLL6			__attribute__ ((section(".ioview"))) = &PPG_PRLL6;
volatile uint8_t  * const IO_PPG_PRLH6			__attribute__ ((section(".ioview"))) = &PPG_PRLH6;
volatile uint8_t  * const IO_PPG_PRLL7			__attribute__ ((section(".ioview"))) = &PPG_PRLL7;
volatile uint8_t  * const IO_PPG_PRLH7			__attribute__ ((section(".ioview"))) = &PPG_PRLH7;
volatile uint8_t  * const IO_PPG_GATEC4 		__attribute__ ((section(".ioview"))) = &PPG_GATEC4;
volatile uint8_t  * const IO_PPG_PPGC9			__attribute__ ((section(".ioview"))) = &PPG_PPGC9;
volatile uint8_t  * const IO_PPG_PPGC8			__attribute__ ((section(".ioview"))) = &PPG_PPGC8;
volatile uint8_t  * const IO_PPG_PPGC11 		__attribute__ ((section(".ioview"))) = &PPG_PPGC11;
volatile uint8_t  * const IO_PPG_PPGC10 		__attribute__ ((section(".ioview"))) = &PPG_PPGC10;
volatile uint8_t  * const IO_PPG_PRLL8			__attribute__ ((section(".ioview"))) = &PPG_PRLL8;
volatile uint8_t  * const IO_PPG_PRLH8			__attribute__ ((section(".ioview"))) = &PPG_PRLH8;
volatile uint8_t  * const IO_PPG_PRLL9			__attribute__ ((section(".ioview"))) = &PPG_PRLL9;
volatile uint8_t  * const IO_PPG_PRLH9			__attribute__ ((section(".ioview"))) = &PPG_PRLH9;
volatile uint8_t  * const IO_PPG_PRLL10 		__attribute__ ((section(".ioview"))) = &PPG_PRLL10;
volatile uint8_t  * const IO_PPG_PRLH10 		__attribute__ ((section(".ioview"))) = &PPG_PRLH10;
volatile uint8_t  * const IO_PPG_PRLL11 		__attribute__ ((section(".ioview"))) = &PPG_PRLL11;
volatile uint8_t  * const IO_PPG_PRLH11 		__attribute__ ((section(".ioview"))) = &PPG_PRLH11;
volatile uint8_t  * const IO_PPG_GATEC8 		__attribute__ ((section(".ioview"))) = &PPG_GATEC8;
volatile uint8_t  * const IO_PPG_PPGC13 		__attribute__ ((section(".ioview"))) = &PPG_PPGC13;
volatile uint8_t  * const IO_PPG_PPGC12 		__attribute__ ((section(".ioview"))) = &PPG_PPGC12;
volatile uint8_t  * const IO_PPG_PPGC15 		__attribute__ ((section(".ioview"))) = &PPG_PPGC15;
volatile uint8_t  * const IO_PPG_PPGC14 		__attribute__ ((section(".ioview"))) = &PPG_PPGC14;
volatile uint8_t  * const IO_PPG_PRLL12 		__attribute__ ((section(".ioview"))) = &PPG_PRLL12;
volatile uint8_t  * const IO_PPG_PRLH12 		__attribute__ ((section(".ioview"))) = &PPG_PRLH12;
volatile uint8_t  * const IO_PPG_PRLL13 		__attribute__ ((section(".ioview"))) = &PPG_PRLL13;
volatile uint8_t  * const IO_PPG_PRLH13 		__attribute__ ((section(".ioview"))) = &PPG_PRLH13;
volatile uint8_t  * const IO_PPG_PRLL14 		__attribute__ ((section(".ioview"))) = &PPG_PRLL14;
volatile uint8_t  * const IO_PPG_PRLH14 		__attribute__ ((section(".ioview"))) = &PPG_PRLH14;
volatile uint8_t  * const IO_PPG_PRLL15 		__attribute__ ((section(".ioview"))) = &PPG_PRLL15;
volatile uint8_t  * const IO_PPG_PRLH15 		__attribute__ ((section(".ioview"))) = &PPG_PRLH15;
volatile uint8_t  * const IO_PPG_GATEC12		__attribute__ ((section(".ioview"))) = &PPG_GATEC12;
volatile uint8_t  * const IO_PPG_PPGC17 		__attribute__ ((section(".ioview"))) = &PPG_PPGC17;
volatile uint8_t  * const IO_PPG_PPGC16 		__attribute__ ((section(".ioview"))) = &PPG_PPGC16;
volatile uint8_t  * const IO_PPG_PPGC19 		__attribute__ ((section(".ioview"))) = &PPG_PPGC19;
volatile uint8_t  * const IO_PPG_PPGC18 		__attribute__ ((section(".ioview"))) = &PPG_PPGC18;
volatile uint8_t  * const IO_PPG_PRLL16 		__attribute__ ((section(".ioview"))) = &PPG_PRLL16;
volatile uint8_t  * const IO_PPG_PRLH16 		__attribute__ ((section(".ioview"))) = &PPG_PRLH16;
volatile uint8_t  * const IO_PPG_PRLH17 		__attribute__ ((section(".ioview"))) = &PPG_PRLH17;
volatile uint8_t  * const IO_PPG_PRLL18 		__attribute__ ((section(".ioview"))) = &PPG_PRLL18;
volatile uint8_t  * const IO_PPG_PRLH18 		__attribute__ ((section(".ioview"))) = &PPG_PRLH18;
volatile uint8_t  * const IO_PPG_PRLL19 		__attribute__ ((section(".ioview"))) = &PPG_PRLL19;
volatile uint8_t  * const IO_PPG_PRLH19 		__attribute__ ((section(".ioview"))) = &PPG_PRLH19;
volatile uint8_t  * const IO_PPG_GATEC16		__attribute__ ((section(".ioview"))) = &PPG_GATEC16;
volatile uint8_t  * const IO_PPG_PPGC21 		__attribute__ ((section(".ioview"))) = &PPG_PPGC21;
volatile uint8_t  * const IO_PPG_PPGC20 		__attribute__ ((section(".ioview"))) = &PPG_PPGC20;
volatile uint8_t  * const IO_PPG_PPGC23 		__attribute__ ((section(".ioview"))) = &PPG_PPGC23;
volatile uint8_t  * const IO_PPG_PPGC22 		__attribute__ ((section(".ioview"))) = &PPG_PPGC22;
volatile uint8_t  * const IO_PPG_PRLL20 		__attribute__ ((section(".ioview"))) = &PPG_PRLL20;
volatile uint8_t  * const IO_PPG_PRLH20 		__attribute__ ((section(".ioview"))) = &PPG_PRLH20;
volatile uint8_t  * const IO_PPG_PRLL21 		__attribute__ ((section(".ioview"))) = &PPG_PRLL21;
volatile uint8_t  * const IO_PPG_PRLH21 		__attribute__ ((section(".ioview"))) = &PPG_PRLH21;
volatile uint8_t  * const IO_PPG_PRLL22 		__attribute__ ((section(".ioview"))) = &PPG_PRLL22;
volatile uint8_t  * const IO_PPG_PRLH22 		__attribute__ ((section(".ioview"))) = &PPG_PRLH22;
volatile uint8_t  * const IO_PPG_PRLL23 		__attribute__ ((section(".ioview"))) = &PPG_PRLL23;
volatile uint8_t  * const IO_PPG_PRLH23 		__attribute__ ((section(".ioview"))) = &PPG_PRLH23;
volatile uint8_t  * const IO_PPG_GATEC20		__attribute__ ((section(".ioview"))) = &PPG_GATEC20;


/* IO_BaseTimer ch0-15 */
volatile uint16_t * const IO_BT0_PCSR			__attribute__ ((section(".ioview"))) = &BT0_PCSR;
volatile uint16_t * const IO_BT0_PRLL			__attribute__ ((section(".ioview"))) = &BT0_PRLL;
volatile uint16_t * const IO_BT0_PDUT			__attribute__ ((section(".ioview"))) = &BT0_PDUT;
volatile uint16_t * const IO_BT0_PRLH			__attribute__ ((section(".ioview"))) = &BT0_PRLH;
volatile uint16_t * const IO_BT0_DTBF			__attribute__ ((section(".ioview"))) = &BT0_DTBF;
volatile uint16_t * const IO_BT0_TMR 			__attribute__ ((section(".ioview"))) = &BT0_TMR;
volatile uint16_t * const IO_BT0_TMCR			__attribute__ ((section(".ioview"))) = &BT0_TMCR;
volatile uint8_t  * const IO_BT0_STC 			__attribute__ ((section(".ioview"))) = &BT0_STC;
volatile uint8_t  * const IO_BT0_TMCR2			__attribute__ ((section(".ioview"))) = &BT0_TMCR2;

volatile uint16_t * const IO_BT1_PCSR			__attribute__ ((section(".ioview"))) = &BT1_PCSR;
volatile uint16_t * const IO_BT1_PRLL			__attribute__ ((section(".ioview"))) = &BT1_PRLL;
volatile uint16_t * const IO_BT1_PDUT			__attribute__ ((section(".ioview"))) = &BT1_PDUT;
volatile uint16_t * const IO_BT1_PRLH			__attribute__ ((section(".ioview"))) = &BT1_PRLH;
volatile uint16_t * const IO_BT1_DTBF			__attribute__ ((section(".ioview"))) = &BT1_DTBF;
volatile uint16_t * const IO_BT1_TMR 			__attribute__ ((section(".ioview"))) = &BT1_TMR;
volatile uint16_t * const IO_BT1_TMCR			__attribute__ ((section(".ioview"))) = &BT1_TMCR;
volatile uint8_t  * const IO_BT1_STC 			__attribute__ ((section(".ioview"))) = &BT1_STC;
volatile uint8_t  * const IO_BT1_TMCR2			__attribute__ ((section(".ioview"))) = &BT1_TMCR2;
 
volatile uint16_t * const IO_BT2_PCSR			__attribute__ ((section(".ioview"))) = &BT2_PCSR;
volatile uint16_t * const IO_BT2_PRLL			__attribute__ ((section(".ioview"))) = &BT2_PRLL;
volatile uint16_t * const IO_BT2_PDUT			__attribute__ ((section(".ioview"))) = &BT2_PDUT;
volatile uint16_t * const IO_BT2_PRLH			__attribute__ ((section(".ioview"))) = &BT2_PRLH;
volatile uint16_t * const IO_BT2_DTBF			__attribute__ ((section(".ioview"))) = &BT2_DTBF;
volatile uint16_t * const IO_BT2_TMR 			__attribute__ ((section(".ioview"))) = &BT2_TMR;
volatile uint16_t * const IO_BT2_TMCR			__attribute__ ((section(".ioview"))) = &BT2_TMCR;
volatile uint8_t  * const IO_BT2_STC 			__attribute__ ((section(".ioview"))) = &BT2_STC;
volatile uint8_t  * const IO_BT2_TMCR2			__attribute__ ((section(".ioview"))) = &BT2_TMCR2;
  
volatile uint16_t * const IO_BT3_PCSR			__attribute__ ((section(".ioview"))) = &BT3_PCSR;
volatile uint16_t * const IO_BT3_PRLL			__attribute__ ((section(".ioview"))) = &BT3_PRLL;
volatile uint16_t * const IO_BT3_PDUT			__attribute__ ((section(".ioview"))) = &BT3_PDUT;
volatile uint16_t * const IO_BT3_PRLH			__attribute__ ((section(".ioview"))) = &BT3_PRLH;
volatile uint16_t * const IO_BT3_DTBF			__attribute__ ((section(".ioview"))) = &BT3_DTBF;
volatile uint16_t * const IO_BT3_TMR 			__attribute__ ((section(".ioview"))) = &BT3_TMR;
volatile uint16_t * const IO_BT3_TMCR			__attribute__ ((section(".ioview"))) = &BT3_TMCR;
volatile uint8_t  * const IO_BT3_STC 			__attribute__ ((section(".ioview"))) = &BT3_STC;
volatile uint8_t  * const IO_BT3_TMCR2			__attribute__ ((section(".ioview"))) = &BT3_TMCR2;
 
volatile uint16_t * const IO_BT4_PCSR			__attribute__ ((section(".ioview"))) = &BT4_PCSR;
volatile uint16_t * const IO_BT4_PRLL			__attribute__ ((section(".ioview"))) = &BT4_PRLL;
volatile uint16_t * const IO_BT4_PDUT			__attribute__ ((section(".ioview"))) = &BT4_PDUT;
volatile uint16_t * const IO_BT4_PRLH			__attribute__ ((section(".ioview"))) = &BT4_PRLH;
volatile uint16_t * const IO_BT4_DTBF			__attribute__ ((section(".ioview"))) = &BT4_DTBF;
volatile uint16_t * const IO_BT4_TMR 			__attribute__ ((section(".ioview"))) = &BT4_TMR;
volatile uint16_t * const IO_BT4_TMCR			__attribute__ ((section(".ioview"))) = &BT4_TMCR;
volatile uint8_t  * const IO_BT4_STC 			__attribute__ ((section(".ioview"))) = &BT4_STC;
volatile uint8_t  * const IO_BT4_TMCR2			__attribute__ ((section(".ioview"))) = &BT4_TMCR2;
 
volatile uint16_t * const IO_BT5_PCSR			__attribute__ ((section(".ioview"))) = &BT5_PCSR;
volatile uint16_t * const IO_BT5_PRLL			__attribute__ ((section(".ioview"))) = &BT5_PRLL;
volatile uint16_t * const IO_BT5_PDUT			__attribute__ ((section(".ioview"))) = &BT5_PDUT;
volatile uint16_t * const IO_BT5_PRLH			__attribute__ ((section(".ioview"))) = &BT5_PRLH;
volatile uint16_t * const IO_BT5_DTBF			__attribute__ ((section(".ioview"))) = &BT5_DTBF;
volatile uint16_t * const IO_BT5_TMR 			__attribute__ ((section(".ioview"))) = &BT5_TMR;
volatile uint16_t * const IO_BT5_TMCR			__attribute__ ((section(".ioview"))) = &BT5_TMCR;
volatile uint8_t  * const IO_BT5_STC 			__attribute__ ((section(".ioview"))) = &BT5_STC;
volatile uint8_t  * const IO_BT5_TMCR2			__attribute__ ((section(".ioview"))) = &BT5_TMCR2;
  
volatile uint16_t * const IO_BT6_PCSR			__attribute__ ((section(".ioview"))) = &BT6_PCSR;
volatile uint16_t * const IO_BT6_PRLL			__attribute__ ((section(".ioview"))) = &BT6_PRLL;
volatile uint16_t * const IO_BT6_PDUT			__attribute__ ((section(".ioview"))) = &BT6_PDUT;
volatile uint16_t * const IO_BT6_PRLH			__attribute__ ((section(".ioview"))) = &BT6_PRLH;
volatile uint16_t * const IO_BT6_DTBF			__attribute__ ((section(".ioview"))) = &BT6_DTBF;
volatile uint16_t * const IO_BT6_TMR 			__attribute__ ((section(".ioview"))) = &BT6_TMR;
volatile uint16_t * const IO_BT6_TMCR			__attribute__ ((section(".ioview"))) = &BT6_TMCR;
volatile uint8_t  * const IO_BT6_STC 			__attribute__ ((section(".ioview"))) = &BT6_STC;
volatile uint8_t  * const IO_BT6_TMCR2			__attribute__ ((section(".ioview"))) = &BT6_TMCR2;
 
volatile uint16_t * const IO_BT7_PCSR			__attribute__ ((section(".ioview"))) = &BT7_PCSR;
volatile uint16_t * const IO_BT7_PRLL			__attribute__ ((section(".ioview"))) = &BT7_PRLL;
volatile uint16_t * const IO_BT7_PDUT			__attribute__ ((section(".ioview"))) = &BT7_PDUT;
volatile uint16_t * const IO_BT7_PRLH			__attribute__ ((section(".ioview"))) = &BT7_PRLH;
volatile uint16_t * const IO_BT7_DTBF			__attribute__ ((section(".ioview"))) = &BT7_DTBF;
volatile uint16_t * const IO_BT7_TMR 			__attribute__ ((section(".ioview"))) = &BT7_TMR;
volatile uint16_t * const IO_BT7_TMCR			__attribute__ ((section(".ioview"))) = &BT7_TMCR;
volatile uint8_t  * const IO_BT7_STC 			__attribute__ ((section(".ioview"))) = &BT7_STC;
volatile uint8_t  * const IO_BT7_TMCR2			__attribute__ ((section(".ioview"))) = &BT7_TMCR2;
  
volatile uint16_t * const IO_BT8_PCSR			__attribute__ ((section(".ioview"))) = &BT8_PCSR;
volatile uint16_t * const IO_BT8_PRLL			__attribute__ ((section(".ioview"))) = &BT8_PRLL;
volatile uint16_t * const IO_BT8_PDUT			__attribute__ ((section(".ioview"))) = &BT8_PDUT;
volatile uint16_t * const IO_BT8_PRLH			__attribute__ ((section(".ioview"))) = &BT8_PRLH;
volatile uint16_t * const IO_BT8_DTBF			__attribute__ ((section(".ioview"))) = &BT8_DTBF;
volatile uint16_t * const IO_BT8_TMR 			__attribute__ ((section(".ioview"))) = &BT8_TMR;
volatile uint16_t * const IO_BT8_TMCR			__attribute__ ((section(".ioview"))) = &BT8_TMCR;
volatile uint8_t  * const IO_BT8_STC 			__attribute__ ((section(".ioview"))) = &BT8_STC;
volatile uint8_t  * const IO_BT8_TMCR2			__attribute__ ((section(".ioview"))) = &BT8_TMCR2;

volatile uint16_t * const IO_BT9_PCSR			__attribute__ ((section(".ioview"))) = &BT9_PCSR;
volatile uint16_t * const IO_BT9_PRLL			__attribute__ ((section(".ioview"))) = &BT9_PRLL;
volatile uint16_t * const IO_BT9_PDUT			__attribute__ ((section(".ioview"))) = &BT9_PDUT;
volatile uint16_t * const IO_BT9_PRLH			__attribute__ ((section(".ioview"))) = &BT9_PRLH;
volatile uint16_t * const IO_BT9_DTBF			__attribute__ ((section(".ioview"))) = &BT9_DTBF;
volatile uint16_t * const IO_BT9_TMR 			__attribute__ ((section(".ioview"))) = &BT9_TMR;
volatile uint16_t * const IO_BT9_TMCR			__attribute__ ((section(".ioview"))) = &BT9_TMCR;
volatile uint8_t  * const IO_BT9_STC 			__attribute__ ((section(".ioview"))) = &BT9_STC;
volatile uint8_t  * const IO_BT9_TMCR2			__attribute__ ((section(".ioview"))) = &BT9_TMCR2;
  
volatile uint16_t * const IO_BT10_PCSR			__attribute__ ((section(".ioview"))) = &BT10_PCSR;
volatile uint16_t * const IO_BT10_PRLL			__attribute__ ((section(".ioview"))) = &BT10_PRLL;
volatile uint16_t * const IO_BT10_PDUT			__attribute__ ((section(".ioview"))) = &BT10_PDUT;
volatile uint16_t * const IO_BT10_PRLH			__attribute__ ((section(".ioview"))) = &BT10_PRLH;
volatile uint16_t * const IO_BT10_DTBF			__attribute__ ((section(".ioview"))) = &BT10_DTBF;
volatile uint16_t * const IO_BT10_TMR			__attribute__ ((section(".ioview"))) = &BT10_TMR;
volatile uint16_t * const IO_BT10_TMCR			__attribute__ ((section(".ioview"))) = &BT10_TMCR;
volatile uint8_t  * const IO_BT10_STC			__attribute__ ((section(".ioview"))) = &BT10_STC;
volatile uint8_t  * const IO_BT10_TMCR2 		__attribute__ ((section(".ioview"))) = &BT10_TMCR2;

volatile uint16_t * const IO_BT11_PCSR			__attribute__ ((section(".ioview"))) = &BT11_PCSR;
volatile uint16_t * const IO_BT11_PRLL			__attribute__ ((section(".ioview"))) = &BT11_PRLL;
volatile uint16_t * const IO_BT11_PDUT			__attribute__ ((section(".ioview"))) = &BT11_PDUT;
volatile uint16_t * const IO_BT11_PRLH			__attribute__ ((section(".ioview"))) = &BT11_PRLH;
volatile uint16_t * const IO_BT11_DTBF			__attribute__ ((section(".ioview"))) = &BT11_DTBF;
volatile uint16_t * const IO_BT11_TMR			__attribute__ ((section(".ioview"))) = &BT11_TMR;
volatile uint16_t * const IO_BT11_TMCR			__attribute__ ((section(".ioview"))) = &BT11_TMCR;
volatile uint8_t  * const IO_BT11_STC			__attribute__ ((section(".ioview"))) = &BT11_STC;
volatile uint8_t  * const IO_BT11_TMCR2 		__attribute__ ((section(".ioview"))) = &BT11_TMCR2;
 
volatile uint16_t * const IO_BT12_PCSR			__attribute__ ((section(".ioview"))) = &BT12_PCSR;
volatile uint16_t * const IO_BT12_PRLL			__attribute__ ((section(".ioview"))) = &BT12_PRLL;
volatile uint16_t * const IO_BT12_PDUT			__attribute__ ((section(".ioview"))) = &BT12_PDUT;
volatile uint16_t * const IO_BT12_PRLH			__attribute__ ((section(".ioview"))) = &BT12_PRLH;
volatile uint16_t * const IO_BT12_DTBF			__attribute__ ((section(".ioview"))) = &BT12_DTBF;
volatile uint16_t * const IO_BT12_TMR			__attribute__ ((section(".ioview"))) = &BT12_TMR;
volatile uint16_t * const IO_BT12_TMCR			__attribute__ ((section(".ioview"))) = &BT12_TMCR;
volatile uint8_t  * const IO_BT12_STC			__attribute__ ((section(".ioview"))) = &BT12_STC;
volatile uint8_t  * const IO_BT12_TMCR2 		__attribute__ ((section(".ioview"))) = &BT12_TMCR2;

volatile uint16_t * const IO_BT13_PCSR			__attribute__ ((section(".ioview"))) = &BT13_PCSR;
volatile uint16_t * const IO_BT13_PRLL			__attribute__ ((section(".ioview"))) = &BT13_PRLL;
volatile uint16_t * const IO_BT13_PDUT			__attribute__ ((section(".ioview"))) = &BT13_PDUT;
volatile uint16_t * const IO_BT13_PRLH			__attribute__ ((section(".ioview"))) = &BT13_PRLH;
volatile uint16_t * const IO_BT13_DTBF			__attribute__ ((section(".ioview"))) = &BT13_DTBF;
volatile uint16_t * const IO_BT13_TMR			__attribute__ ((section(".ioview"))) = &BT13_TMR;
volatile uint16_t * const IO_BT13_TMCR			__attribute__ ((section(".ioview"))) = &BT13_TMCR;
volatile uint8_t  * const IO_BT13_STC			__attribute__ ((section(".ioview"))) = &BT13_STC;
volatile uint8_t  * const IO_BT13_TMCR2 		__attribute__ ((section(".ioview"))) = &BT13_TMCR2;

volatile uint16_t * const IO_BT14_PCSR			__attribute__ ((section(".ioview"))) = &BT14_PCSR;
volatile uint16_t * const IO_BT14_PRLL			__attribute__ ((section(".ioview"))) = &BT14_PRLL;
volatile uint16_t * const IO_BT14_PDUT			__attribute__ ((section(".ioview"))) = &BT14_PDUT;
volatile uint16_t * const IO_BT14_PRLH			__attribute__ ((section(".ioview"))) = &BT14_PRLH;
volatile uint16_t * const IO_BT14_DTBF			__attribute__ ((section(".ioview"))) = &BT14_DTBF;
volatile uint16_t * const IO_BT14_TMR			__attribute__ ((section(".ioview"))) = &BT14_TMR;
volatile uint16_t * const IO_BT14_TMCR			__attribute__ ((section(".ioview"))) = &BT14_TMCR;
volatile uint8_t  * const IO_BT14_STC			__attribute__ ((section(".ioview"))) = &BT14_STC;
volatile uint8_t  * const IO_BT14_TMCR2 		__attribute__ ((section(".ioview"))) = &BT14_TMCR2;

volatile uint16_t * const IO_BT15_PCSR			__attribute__ ((section(".ioview"))) = &BT15_PCSR;
volatile uint16_t * const IO_BT15_PRLL			__attribute__ ((section(".ioview"))) = &BT15_PRLL;
volatile uint16_t * const IO_BT15_PDUT			__attribute__ ((section(".ioview"))) = &BT15_PDUT;
volatile uint16_t * const IO_BT15_PRLH			__attribute__ ((section(".ioview"))) = &BT15_PRLH;
volatile uint16_t * const IO_BT15_DTBF			__attribute__ ((section(".ioview"))) = &BT15_DTBF;
volatile uint16_t * const IO_BT15_TMR			__attribute__ ((section(".ioview"))) = &BT15_TMR;
volatile uint16_t * const IO_BT15_TMCR			__attribute__ ((section(".ioview"))) = &BT15_TMCR;
volatile uint8_t  * const IO_BT15_STC			__attribute__ ((section(".ioview"))) = &BT15_STC;
volatile uint8_t  * const IO_BT15_TMCR2 		__attribute__ ((section(".ioview"))) = &BT15_TMCR2;

volatile uint8_t  * const IO_BTSEL0123			__attribute__ ((section(".ioview"))) = &BTSEL0123;
volatile uint8_t  * const IO_BTSEL4567			__attribute__ ((section(".ioview"))) = &BTSEL4567;
volatile uint8_t  * const IO_BTSEL89AB			__attribute__ ((section(".ioview"))) = &BTSEL89AB;
volatile uint8_t  * const IO_BTSELCDEF			__attribute__ ((section(".ioview"))) = &BTSELCDEF;


/* IO_Softwrtup */
volatile uint16_t * const IO_BTSSSR	 			__attribute__ ((section(".ioview"))) = &BTSSSR;


/* IO_QPRC ch0-2 */ 
volatile uint16_t * const IO_QPRC0_QPCR 		__attribute__ ((section(".ioview"))) = &QPRC0_QPCR;
volatile uint16_t * const IO_QPRC0_QRCR 		__attribute__ ((section(".ioview"))) = &QPRC0_QRCR;
volatile uint16_t * const IO_QPRC0_QPCCR		__attribute__ ((section(".ioview"))) = &QPRC0_QPCCR;
volatile uint16_t * const IO_QPRC0_QPRCR		__attribute__ ((section(".ioview"))) = &QPRC0_QPRCR;
volatile uint16_t * const IO_QPRC0_QMPR 		__attribute__ ((section(".ioview"))) = &QPRC0_QMPR;
volatile uint8_t  * const IO_QPRC0_QICRL		__attribute__ ((section(".ioview"))) = &QPRC0_QICRL;
volatile uint8_t  * const IO_QPRC0_QICRH		__attribute__ ((section(".ioview"))) = &QPRC0_QICRH;
volatile uint8_t  * const IO_QPRC0_QCRL 		__attribute__ ((section(".ioview"))) = &QPRC0_QCRL;
volatile uint8_t  * const IO_QPRC0_QCRH 		__attribute__ ((section(".ioview"))) = &QPRC0_QCRH;
volatile uint16_t * const IO_QPRC0_QECR 		__attribute__ ((section(".ioview"))) = &QPRC0_QECR;
volatile uint16_t * const IO_QPRC0_QRCRR		__attribute__ ((section(".ioview"))) = &QPRC0_QRCRR;
volatile uint16_t * const IO_QPRC0_QPCRR		__attribute__ ((section(".ioview"))) = &QPRC0_QPCRR;
 
volatile uint16_t * const IO_QPRC1_QPCR 		__attribute__ ((section(".ioview"))) = &QPRC1_QPCR;
volatile uint16_t * const IO_QPRC1_QRCR 		__attribute__ ((section(".ioview"))) = &QPRC1_QRCR;
volatile uint16_t * const IO_QPRC1_QPCCR		__attribute__ ((section(".ioview"))) = &QPRC1_QPCCR;
volatile uint16_t * const IO_QPRC1_QPRCR		__attribute__ ((section(".ioview"))) = &QPRC1_QPRCR;
volatile uint16_t * const IO_QPRC1_QMPR 		__attribute__ ((section(".ioview"))) = &QPRC1_QMPR;
volatile uint8_t  * const IO_QPRC1_QICRL		__attribute__ ((section(".ioview"))) = &QPRC1_QICRL;
volatile uint8_t  * const IO_QPRC1_QICRH		__attribute__ ((section(".ioview"))) = &QPRC1_QICRH;
volatile uint8_t  * const IO_QPRC1_QCRL 		__attribute__ ((section(".ioview"))) = &QPRC1_QCRL;
volatile uint8_t  * const IO_QPRC1_QCRH 		__attribute__ ((section(".ioview"))) = &QPRC1_QCRH;
volatile uint16_t * const IO_QPRC1_QECR 		__attribute__ ((section(".ioview"))) = &QPRC1_QECR;
volatile uint16_t * const IO_QPRC1_QRCRR		__attribute__ ((section(".ioview"))) = &QPRC1_QRCRR;
volatile uint16_t * const IO_QPRC1_QPCRR		__attribute__ ((section(".ioview"))) = &QPRC1_QPCRR;
 
volatile uint16_t * const IO_QPRC2_QPCR 		__attribute__ ((section(".ioview"))) = &QPRC2_QPCR;
volatile uint16_t * const IO_QPRC2_QRCR 		__attribute__ ((section(".ioview"))) = &QPRC2_QRCR;
volatile uint16_t * const IO_QPRC2_QPCCR		__attribute__ ((section(".ioview"))) = &QPRC2_QPCCR;
volatile uint16_t * const IO_QPRC2_QPRCR		__attribute__ ((section(".ioview"))) = &QPRC2_QPRCR;
volatile uint16_t * const IO_QPRC2_QMPR 		__attribute__ ((section(".ioview"))) = &QPRC2_QMPR;
volatile uint8_t  * const IO_QPRC2_QICRL		__attribute__ ((section(".ioview"))) = &QPRC2_QICRL;
volatile uint8_t  * const IO_QPRC2_QICRH		__attribute__ ((section(".ioview"))) = &QPRC2_QICRH;
volatile uint8_t  * const IO_QPRC2_QCRL 		__attribute__ ((section(".ioview"))) = &QPRC2_QCRL;
volatile uint8_t  * const IO_QPRC2_QCRH 		__attribute__ ((section(".ioview"))) = &QPRC2_QCRH;
volatile uint16_t * const IO_QPRC2_QECR 		__attribute__ ((section(".ioview"))) = &QPRC2_QECR;
volatile uint16_t * const IO_QPRC2_QRCRR		__attribute__ ((section(".ioview"))) = &QPRC2_QRCRR;
volatile uint16_t * const IO_QPRC2_QPCRR		__attribute__ ((section(".ioview"))) = &QPRC2_QPCRR;


/* IO_12bit ADC unit0-2 */
volatile uint8_t  * const IO_ADC0_ADSR			__attribute__ ((section(".ioview"))) = &ADC0_ADSR;
volatile uint8_t  * const IO_ADC0_ADCR			__attribute__ ((section(".ioview"))) = &ADC0_ADCR;
volatile uint8_t  * const IO_ADC0_SFNS			__attribute__ ((section(".ioview"))) = &ADC0_SFNS;
volatile uint8_t  * const IO_ADC0_SCCR			__attribute__ ((section(".ioview"))) = &ADC0_SCCR;
volatile uint32_t * const IO_ADC0_SCFD			__attribute__ ((section(".ioview"))) = &ADC0_SCFD;
volatile uint8_t  * const IO_ADC0_SCIS2			__attribute__ ((section(".ioview"))) = &ADC0_SCIS2;
volatile uint8_t  * const IO_ADC0_SCIS3			__attribute__ ((section(".ioview"))) = &ADC0_SCIS3;
volatile uint8_t  * const IO_ADC0_SCIS0			__attribute__ ((section(".ioview"))) = &ADC0_SCIS0;
volatile uint8_t  * const IO_ADC0_SCIS1			__attribute__ ((section(".ioview"))) = &ADC0_SCIS1;
volatile uint8_t  * const IO_ADC0_PFNS			__attribute__ ((section(".ioview"))) = &ADC0_PFNS;
volatile uint8_t  * const IO_ADC0_PCCR			__attribute__ ((section(".ioview"))) = &ADC0_PCCR;
volatile uint32_t * const IO_ADC0_PCFD			__attribute__ ((section(".ioview"))) = &ADC0_PCFD;
volatile uint8_t  * const IO_ADC0_PCIS			__attribute__ ((section(".ioview"))) = &ADC0_PCIS;
volatile uint8_t  * const IO_ADC0_CMPCR			__attribute__ ((section(".ioview"))) = &ADC0_CMPCR;
volatile uint16_t * const IO_ADC0_CMPD			__attribute__ ((section(".ioview"))) = &ADC0_CMPD;
volatile uint8_t  * const IO_ADC0_ADSS2			__attribute__ ((section(".ioview"))) = &ADC0_ADSS2;
volatile uint8_t  * const IO_ADC0_ADSS3			__attribute__ ((section(".ioview"))) = &ADC0_ADSS3;
volatile uint8_t  * const IO_ADC0_ADSS0			__attribute__ ((section(".ioview"))) = &ADC0_ADSS0;
volatile uint8_t  * const IO_ADC0_ADSS1			__attribute__ ((section(".ioview"))) = &ADC0_ADSS1;
volatile uint8_t  * const IO_ADC0_ADST1			__attribute__ ((section(".ioview"))) = &ADC0_ADST1;
volatile uint8_t  * const IO_ADC0_ADST0			__attribute__ ((section(".ioview"))) = &ADC0_ADST0;
volatile uint8_t  * const IO_ADC0_ADCT			__attribute__ ((section(".ioview"))) = &ADC0_ADCT;
volatile uint8_t  * const IO_ADC0_PRTSL			__attribute__ ((section(".ioview"))) = &ADC0_PRTSL;
volatile uint8_t  * const IO_ADC0_SCTSL			__attribute__ ((section(".ioview"))) = &ADC0_SCTSL;
volatile uint8_t  * const IO_ADC0_ADCEN			__attribute__ ((section(".ioview"))) = &ADC0_ADCEN;
     
volatile uint8_t  * const IO_ADC1_ADSR			__attribute__ ((section(".ioview"))) = &ADC1_ADSR;
volatile uint8_t  * const IO_ADC1_ADCR			__attribute__ ((section(".ioview"))) = &ADC1_ADCR;
volatile uint8_t  * const IO_ADC1_SFNS			__attribute__ ((section(".ioview"))) = &ADC1_SFNS;
volatile uint8_t  * const IO_ADC1_SCCR			__attribute__ ((section(".ioview"))) = &ADC1_SCCR;
volatile uint32_t * const IO_ADC1_SCFD			__attribute__ ((section(".ioview"))) = &ADC1_SCFD;
volatile uint8_t  * const IO_ADC1_SCIS2			__attribute__ ((section(".ioview"))) = &ADC1_SCIS2;
volatile uint8_t  * const IO_ADC1_SCIS3			__attribute__ ((section(".ioview"))) = &ADC1_SCIS3;
volatile uint8_t  * const IO_ADC1_SCIS0			__attribute__ ((section(".ioview"))) = &ADC1_SCIS0;
volatile uint8_t  * const IO_ADC1_SCIS1			__attribute__ ((section(".ioview"))) = &ADC1_SCIS1;
volatile uint8_t  * const IO_ADC1_PFNS			__attribute__ ((section(".ioview"))) = &ADC1_PFNS;
volatile uint8_t  * const IO_ADC1_PCCR			__attribute__ ((section(".ioview"))) = &ADC1_PCCR;
volatile uint32_t * const IO_ADC1_PCFD			__attribute__ ((section(".ioview"))) = &ADC1_PCFD;
volatile uint8_t  * const IO_ADC1_PCIS			__attribute__ ((section(".ioview"))) = &ADC1_PCIS;
volatile uint8_t  * const IO_ADC1_CMPCR			__attribute__ ((section(".ioview"))) = &ADC1_CMPCR;
volatile uint16_t * const IO_ADC1_CMPD			__attribute__ ((section(".ioview"))) = &ADC1_CMPD;
volatile uint8_t  * const IO_ADC1_ADSS2			__attribute__ ((section(".ioview"))) = &ADC1_ADSS2;
volatile uint8_t  * const IO_ADC1_ADSS3			__attribute__ ((section(".ioview"))) = &ADC1_ADSS3;
volatile uint8_t  * const IO_ADC1_ADSS0			__attribute__ ((section(".ioview"))) = &ADC1_ADSS0;
volatile uint8_t  * const IO_ADC1_ADSS1			__attribute__ ((section(".ioview"))) = &ADC1_ADSS1;
volatile uint8_t  * const IO_ADC1_ADST1			__attribute__ ((section(".ioview"))) = &ADC1_ADST1;
volatile uint8_t  * const IO_ADC1_ADST0			__attribute__ ((section(".ioview"))) = &ADC1_ADST0;
volatile uint8_t  * const IO_ADC1_ADCT			__attribute__ ((section(".ioview"))) = &ADC1_ADCT;
volatile uint8_t  * const IO_ADC1_PRTSL			__attribute__ ((section(".ioview"))) = &ADC1_PRTSL;
volatile uint8_t  * const IO_ADC1_SCTSL			__attribute__ ((section(".ioview"))) = &ADC1_SCTSL;
volatile uint8_t  * const IO_ADC1_ADCEN			__attribute__ ((section(".ioview"))) = &ADC1_ADCEN;
     
volatile uint8_t  * const IO_ADC2_ADSR			__attribute__ ((section(".ioview"))) = &ADC2_ADSR;
volatile uint8_t  * const IO_ADC2_ADCR			__attribute__ ((section(".ioview"))) = &ADC2_ADCR;
volatile uint8_t  * const IO_ADC2_SFNS			__attribute__ ((section(".ioview"))) = &ADC2_SFNS;
volatile uint8_t  * const IO_ADC2_SCCR			__attribute__ ((section(".ioview"))) = &ADC2_SCCR;
volatile uint32_t * const IO_ADC2_SCFD			__attribute__ ((section(".ioview"))) = &ADC2_SCFD;
volatile uint8_t  * const IO_ADC2_SCIS2			__attribute__ ((section(".ioview"))) = &ADC2_SCIS2;
volatile uint8_t  * const IO_ADC2_SCIS3			__attribute__ ((section(".ioview"))) = &ADC2_SCIS3;
volatile uint8_t  * const IO_ADC2_SCIS0			__attribute__ ((section(".ioview"))) = &ADC2_SCIS0;
volatile uint8_t  * const IO_ADC2_SCIS1			__attribute__ ((section(".ioview"))) = &ADC2_SCIS1;
volatile uint8_t  * const IO_ADC2_PFNS			__attribute__ ((section(".ioview"))) = &ADC2_PFNS;
volatile uint8_t  * const IO_ADC2_PCCR			__attribute__ ((section(".ioview"))) = &ADC2_PCCR;
volatile uint32_t * const IO_ADC2_PCFD			__attribute__ ((section(".ioview"))) = &ADC2_PCFD;
volatile uint8_t  * const IO_ADC2_PCIS			__attribute__ ((section(".ioview"))) = &ADC2_PCIS;
volatile uint8_t  * const IO_ADC2_CMPCR			__attribute__ ((section(".ioview"))) = &ADC2_CMPCR;
volatile uint16_t * const IO_ADC2_CMPD			__attribute__ ((section(".ioview"))) = &ADC2_CMPD;
volatile uint8_t  * const IO_ADC2_ADSS2			__attribute__ ((section(".ioview"))) = &ADC2_ADSS2;
volatile uint8_t  * const IO_ADC2_ADSS3			__attribute__ ((section(".ioview"))) = &ADC2_ADSS3;
volatile uint8_t  * const IO_ADC2_ADSS0			__attribute__ ((section(".ioview"))) = &ADC2_ADSS0;
volatile uint8_t  * const IO_ADC2_ADSS1			__attribute__ ((section(".ioview"))) = &ADC2_ADSS1;
volatile uint8_t  * const IO_ADC2_ADST1			__attribute__ ((section(".ioview"))) = &ADC2_ADST1;
volatile uint8_t  * const IO_ADC2_ADST0			__attribute__ ((section(".ioview"))) = &ADC2_ADST0;
volatile uint8_t  * const IO_ADC2_ADCT			__attribute__ ((section(".ioview"))) = &ADC2_ADCT;
volatile uint8_t  * const IO_ADC2_PRTSL			__attribute__ ((section(".ioview"))) = &ADC2_PRTSL;
volatile uint8_t  * const IO_ADC2_SCTSL			__attribute__ ((section(".ioview"))) = &ADC2_SCTSL;
volatile uint8_t  * const IO_ADC2_ADCEN			__attribute__ ((section(".ioview"))) = &ADC2_ADCEN;


/* IO_CR Trim */  
volatile uint8_t  * const IO_MCR_PSR			__attribute__ ((section(".ioview"))) = &MCR_PSR;
volatile uint16_t * const IO_MCR_FTRM			__attribute__ ((section(".ioview"))) = &MCR_FTRM;
volatile uint32_t * const IO_MCR_RLR			__attribute__ ((section(".ioview"))) = &MCR_RLR;
 
/* IO_EXTI */
volatile uint32_t * const IO_EXTI_ENIR 			__attribute__ ((section(".ioview"))) = &EXTI_ENIR;
volatile uint32_t * const IO_EXTI_EIRR 			__attribute__ ((section(".ioview"))) = &EXTI_EIRR;
volatile uint32_t * const IO_EXTI_EICL 			__attribute__ ((section(".ioview"))) = &EXTI_EICL;
volatile uint32_t * const IO_EXTI_ELVR 			__attribute__ ((section(".ioview"))) = &EXTI_ELVR;
volatile uint32_t * const IO_EXTI_ELVR1			__attribute__ ((section(".ioview"))) = &EXTI_ELVR1;
volatile uint8_t  * const IO_EXTI_NMIRR			__attribute__ ((section(".ioview"))) = &EXTI_NMIRR;
volatile uint8_t  * const IO_EXTI_NMICL			__attribute__ ((section(".ioview"))) = &EXTI_NMICL;


/* IO_INT-Req. Read */   
volatile uint32_t * const IO_DRQSEL				__attribute__ ((section(".ioview"))) = &DRQSEL;
volatile uint8_t  * const IO_ODDPKS				__attribute__ ((section(".ioview"))) = &ODDPKS;
volatile uint32_t * const IO_EXC02MON 			__attribute__ ((section(".ioview"))) = &EXC02MON;
volatile uint32_t * const IO_IRQ00MON 			__attribute__ ((section(".ioview"))) = &IRQ00MON;
volatile uint32_t * const IO_IRQ01MON 			__attribute__ ((section(".ioview"))) = &IRQ01MON;
volatile uint32_t * const IO_IRQ02MON 			__attribute__ ((section(".ioview"))) = &IRQ02MON;
volatile uint32_t * const IO_IRQ03MON 			__attribute__ ((section(".ioview"))) = &IRQ03MON;
volatile uint32_t * const IO_IRQ04MON 			__attribute__ ((section(".ioview"))) = &IRQ04MON;
volatile uint32_t * const IO_IRQ05MON 			__attribute__ ((section(".ioview"))) = &IRQ05MON;
volatile uint32_t * const IO_IRQ06MON 			__attribute__ ((section(".ioview"))) = &IRQ06MON;
volatile uint32_t * const IO_IRQ07MON 			__attribute__ ((section(".ioview"))) = &IRQ07MON;
volatile uint32_t * const IO_IRQ08MON 			__attribute__ ((section(".ioview"))) = &IRQ08MON;
volatile uint32_t * const IO_IRQ09MON 			__attribute__ ((section(".ioview"))) = &IRQ09MON;
volatile uint32_t * const IO_IRQ10MON 			__attribute__ ((section(".ioview"))) = &IRQ10MON;
volatile uint32_t * const IO_IRQ11MON 			__attribute__ ((section(".ioview"))) = &IRQ11MON;
volatile uint32_t * const IO_IRQ12MON 			__attribute__ ((section(".ioview"))) = &IRQ12MON;
volatile uint32_t * const IO_IRQ13MON 			__attribute__ ((section(".ioview"))) = &IRQ13MON;
volatile uint32_t * const IO_IRQ14MON 			__attribute__ ((section(".ioview"))) = &IRQ14MON;
volatile uint32_t * const IO_IRQ15MON 			__attribute__ ((section(".ioview"))) = &IRQ15MON;
volatile uint32_t * const IO_IRQ16MON 			__attribute__ ((section(".ioview"))) = &IRQ16MON;
volatile uint32_t * const IO_IRQ17MON 			__attribute__ ((section(".ioview"))) = &IRQ17MON;
volatile uint32_t * const IO_IRQ18MON 			__attribute__ ((section(".ioview"))) = &IRQ18MON;
volatile uint32_t * const IO_IRQ19MON 			__attribute__ ((section(".ioview"))) = &IRQ19MON;
volatile uint32_t * const IO_IRQ20MON 			__attribute__ ((section(".ioview"))) = &IRQ20MON;
volatile uint32_t * const IO_IRQ21MON 			__attribute__ ((section(".ioview"))) = &IRQ21MON;
volatile uint32_t * const IO_IRQ22MON 			__attribute__ ((section(".ioview"))) = &IRQ22MON;
volatile uint32_t * const IO_IRQ23MON 			__attribute__ ((section(".ioview"))) = &IRQ23MON;
volatile uint32_t * const IO_IRQ24MON 			__attribute__ ((section(".ioview"))) = &IRQ24MON;
volatile uint32_t * const IO_IRQ25MON 			__attribute__ ((section(".ioview"))) = &IRQ25MON;
volatile uint32_t * const IO_IRQ26MON 			__attribute__ ((section(".ioview"))) = &IRQ26MON;
volatile uint32_t * const IO_IRQ27MON 			__attribute__ ((section(".ioview"))) = &IRQ27MON;
volatile uint32_t * const IO_IRQ28MON 			__attribute__ ((section(".ioview"))) = &IRQ28MON;
volatile uint32_t * const IO_IRQ29MON 			__attribute__ ((section(".ioview"))) = &IRQ29MON;
volatile uint32_t * const IO_IRQ30MON 			__attribute__ ((section(".ioview"))) = &IRQ30MON;
volatile uint32_t * const IO_IRQ31MON 			__attribute__ ((section(".ioview"))) = &IRQ31MON;
volatile uint32_t * const IO_IRQ32MON 			__attribute__ ((section(".ioview"))) = &IRQ32MON;
volatile uint32_t * const IO_IRQ33MON 			__attribute__ ((section(".ioview"))) = &IRQ33MON;
volatile uint32_t * const IO_IRQ34MON 			__attribute__ ((section(".ioview"))) = &IRQ34MON;
volatile uint32_t * const IO_IRQ35MON 			__attribute__ ((section(".ioview"))) = &IRQ35MON;
volatile uint32_t * const IO_IRQ36MON 			__attribute__ ((section(".ioview"))) = &IRQ36MON;
volatile uint32_t * const IO_IRQ37MON 			__attribute__ ((section(".ioview"))) = &IRQ37MON;
volatile uint32_t * const IO_IRQ38MON 			__attribute__ ((section(".ioview"))) = &IRQ38MON;
volatile uint32_t * const IO_IRQ39MON 			__attribute__ ((section(".ioview"))) = &IRQ39MON;
volatile uint32_t * const IO_IRQ40MON 			__attribute__ ((section(".ioview"))) = &IRQ40MON;
volatile uint32_t * const IO_IRQ41MON 			__attribute__ ((section(".ioview"))) = &IRQ41MON;
volatile uint32_t * const IO_IRQ42MON 			__attribute__ ((section(".ioview"))) = &IRQ42MON;
volatile uint32_t * const IO_IRQ43MON 			__attribute__ ((section(".ioview"))) = &IRQ43MON;
volatile uint32_t * const IO_IRQ44MON 			__attribute__ ((section(".ioview"))) = &IRQ44MON;
volatile uint32_t * const IO_IRQ45MON 			__attribute__ ((section(".ioview"))) = &IRQ45MON;
volatile uint32_t * const IO_IRQ46MON 			__attribute__ ((section(".ioview"))) = &IRQ46MON;
volatile uint32_t * const IO_IRQ47MON 			__attribute__ ((section(".ioview"))) = &IRQ47MON;
volatile uint32_t * const IO_DRQSEL1			__attribute__ ((section(".ioview"))) = &DRQSEL1;
volatile uint32_t * const IO_DQESEL				__attribute__ ((section(".ioview"))) = &DQESEL;
volatile uint8_t  * const IO_ODDPKS1			__attribute__ ((section(".ioview"))) = &ODDPKS1;

/* IO_GPIO */
volatile uint32_t * const IO_PFR0				__attribute__ ((section(".ioview"))) = &PFR0;
volatile uint32_t * const IO_PFR1				__attribute__ ((section(".ioview"))) = &PFR1;
volatile uint32_t * const IO_PFR2				__attribute__ ((section(".ioview"))) = &PFR2;
volatile uint32_t * const IO_PFR3				__attribute__ ((section(".ioview"))) = &PFR3;
volatile uint32_t * const IO_PFR4				__attribute__ ((section(".ioview"))) = &PFR4;
volatile uint32_t * const IO_PFR5				__attribute__ ((section(".ioview"))) = &PFR5;
volatile uint32_t * const IO_PFR6				__attribute__ ((section(".ioview"))) = &PFR6;
volatile uint32_t * const IO_PFR7				__attribute__ ((section(".ioview"))) = &PFR7;
volatile uint32_t * const IO_PFR8				__attribute__ ((section(".ioview"))) = &PFR8;
volatile uint32_t * const IO_PFR9				__attribute__ ((section(".ioview"))) = &PFR9;
volatile uint32_t * const IO_PFRA				__attribute__ ((section(".ioview"))) = &PFRA;
volatile uint32_t * const IO_PFRB				__attribute__ ((section(".ioview"))) = &PFRB;
volatile uint32_t * const IO_PFRC				__attribute__ ((section(".ioview"))) = &PFRC;
volatile uint32_t * const IO_PFRD				__attribute__ ((section(".ioview"))) = &PFRD;
volatile uint32_t * const IO_PFRE				__attribute__ ((section(".ioview"))) = &PFRE;
volatile uint32_t * const IO_PFRF				__attribute__ ((section(".ioview"))) = &PFRF;
volatile uint32_t * const IO_PCR0				__attribute__ ((section(".ioview"))) = &PCR0;
volatile uint32_t * const IO_PCR1				__attribute__ ((section(".ioview"))) = &PCR1;
volatile uint32_t * const IO_PCR2				__attribute__ ((section(".ioview"))) = &PCR2;
volatile uint32_t * const IO_PCR3				__attribute__ ((section(".ioview"))) = &PCR3;
volatile uint32_t * const IO_PCR4				__attribute__ ((section(".ioview"))) = &PCR4;
volatile uint32_t * const IO_PCR5				__attribute__ ((section(".ioview"))) = &PCR5;
volatile uint32_t * const IO_PCR6				__attribute__ ((section(".ioview"))) = &PCR6;
volatile uint32_t * const IO_PCR7				__attribute__ ((section(".ioview"))) = &PCR7;
volatile uint32_t * const IO_PCR8				__attribute__ ((section(".ioview"))) = &PCR8;
volatile uint32_t * const IO_PCR9				__attribute__ ((section(".ioview"))) = &PCR9;
volatile uint32_t * const IO_PCRA				__attribute__ ((section(".ioview"))) = &PCRA;
volatile uint32_t * const IO_PCRB				__attribute__ ((section(".ioview"))) = &PCRB;
volatile uint32_t * const IO_PCRC				__attribute__ ((section(".ioview"))) = &PCRC;
volatile uint32_t * const IO_PCRD				__attribute__ ((section(".ioview"))) = &PCRD;
volatile uint32_t * const IO_PCRE				__attribute__ ((section(".ioview"))) = &PCRE;
volatile uint32_t * const IO_PCRF				__attribute__ ((section(".ioview"))) = &PCRF;
volatile uint32_t * const IO_DDR0				__attribute__ ((section(".ioview"))) = &DDR0;
volatile uint32_t * const IO_DDR1				__attribute__ ((section(".ioview"))) = &DDR1;
volatile uint32_t * const IO_DDR2				__attribute__ ((section(".ioview"))) = &DDR2;
volatile uint32_t * const IO_DDR3				__attribute__ ((section(".ioview"))) = &DDR3;
volatile uint32_t * const IO_DDR4				__attribute__ ((section(".ioview"))) = &DDR4;
volatile uint32_t * const IO_DDR5				__attribute__ ((section(".ioview"))) = &DDR5;
volatile uint32_t * const IO_DDR6				__attribute__ ((section(".ioview"))) = &DDR6;
volatile uint32_t * const IO_DDR7				__attribute__ ((section(".ioview"))) = &DDR7;
volatile uint32_t * const IO_DDR8				__attribute__ ((section(".ioview"))) = &DDR8;
volatile uint32_t * const IO_DDR9				__attribute__ ((section(".ioview"))) = &DDR9;
volatile uint32_t * const IO_DDRA				__attribute__ ((section(".ioview"))) = &DDRA;
volatile uint32_t * const IO_DDRB				__attribute__ ((section(".ioview"))) = &DDRB;
volatile uint32_t * const IO_DDRC				__attribute__ ((section(".ioview"))) = &DDRC;
volatile uint32_t * const IO_DDRD				__attribute__ ((section(".ioview"))) = &DDRD;
volatile uint32_t * const IO_DDRE				__attribute__ ((section(".ioview"))) = &DDRE;
volatile uint32_t * const IO_DDRF				__attribute__ ((section(".ioview"))) = &DDRF;
volatile uint32_t * const IO_PDIR0				__attribute__ ((section(".ioview"))) = &PDIR0;
volatile uint32_t * const IO_PDIR1				__attribute__ ((section(".ioview"))) = &PDIR1;
volatile uint32_t * const IO_PDIR2				__attribute__ ((section(".ioview"))) = &PDIR2;
volatile uint32_t * const IO_PDIR3				__attribute__ ((section(".ioview"))) = &PDIR3;
volatile uint32_t * const IO_PDIR4				__attribute__ ((section(".ioview"))) = &PDIR4;
volatile uint32_t * const IO_PDIR5				__attribute__ ((section(".ioview"))) = &PDIR5;
volatile uint32_t * const IO_PDIR6				__attribute__ ((section(".ioview"))) = &PDIR6;
volatile uint32_t * const IO_PDIR7				__attribute__ ((section(".ioview"))) = &PDIR7;
volatile uint32_t * const IO_PDIR8				__attribute__ ((section(".ioview"))) = &PDIR8;
volatile uint32_t * const IO_PDIR9				__attribute__ ((section(".ioview"))) = &PDIR9;
volatile uint32_t * const IO_PDIRA				__attribute__ ((section(".ioview"))) = &PDIRA;
volatile uint32_t * const IO_PDIRB				__attribute__ ((section(".ioview"))) = &PDIRB;
volatile uint32_t * const IO_PDIRC				__attribute__ ((section(".ioview"))) = &PDIRC;
volatile uint32_t * const IO_PDIRD				__attribute__ ((section(".ioview"))) = &PDIRD;
volatile uint32_t * const IO_PDIRE				__attribute__ ((section(".ioview"))) = &PDIRE;
volatile uint32_t * const IO_PDIRF				__attribute__ ((section(".ioview"))) = &PDIRF;
volatile uint32_t * const IO_PDOR0				__attribute__ ((section(".ioview"))) = &PDOR0;
volatile stc_gpio_pdor0_field_t * const IO_PDOR0_B __attribute__ ((section(".ioview"))) = (stc_gpio_pdor0_field_t *)&PDOR0;
volatile uint32_t * const IO_PDOR1				__attribute__ ((section(".ioview"))) = &PDOR1;
volatile stc_gpio_pdor1_field_t * const IO_PDOR1_B __attribute__ ((section(".ioview"))) = (stc_gpio_pdor1_field_t *)&PDOR1;
volatile uint32_t * const IO_PDOR2				__attribute__ ((section(".ioview"))) = &PDOR2;
volatile stc_gpio_pdor2_field_t * const IO_PDOR2_B __attribute__ ((section(".ioview"))) = (stc_gpio_pdor2_field_t *)&PDOR3;
volatile uint32_t * const IO_PDOR3				__attribute__ ((section(".ioview"))) = &PDOR3;
volatile stc_gpio_pdor3_field_t * const IO_PDOR3_B __attribute__ ((section(".ioview"))) = (stc_gpio_pdor3_field_t *)&PDOR3;
volatile uint32_t * const IO_PDOR4				__attribute__ ((section(".ioview"))) = &PDOR4;
volatile stc_gpio_pdor4_field_t * const IO_PDOR4_B __attribute__ ((section(".ioview"))) = (stc_gpio_pdor4_field_t *)&PDOR4;
volatile uint32_t * const IO_PDOR5				__attribute__ ((section(".ioview"))) = &PDOR5;
volatile stc_gpio_pdor5_field_t * const IO_PDOR5_B __attribute__ ((section(".ioview"))) = (stc_gpio_pdor5_field_t *)&PDOR5;
volatile uint32_t * const IO_PDOR6				__attribute__ ((section(".ioview"))) = &PDOR6;
volatile stc_gpio_pdor6_field_t * const IO_PDOR6_B __attribute__ ((section(".ioview"))) = (stc_gpio_pdor6_field_t *)&PDOR6;
volatile uint32_t * const IO_PDOR7				__attribute__ ((section(".ioview"))) = &PDOR7;
volatile stc_gpio_pdor7_field_t * const IO_PDOR7_B __attribute__ ((section(".ioview"))) = (stc_gpio_pdor7_field_t *)&PDOR7;
volatile uint32_t * const IO_PDOR8				__attribute__ ((section(".ioview"))) = &PDOR8;
volatile stc_gpio_pdor8_field_t * const IO_PDOR8_B __attribute__ ((section(".ioview"))) = (stc_gpio_pdor8_field_t *)&PDOR8;
volatile uint32_t * const IO_PDOR9				__attribute__ ((section(".ioview"))) = &PDOR9;
volatile stc_gpio_pdor9_field_t * const IO_PDOR9_B __attribute__ ((section(".ioview"))) = (stc_gpio_pdor9_field_t *)&PDOR9;
volatile uint32_t * const IO_PDORA				__attribute__ ((section(".ioview"))) = &PDORA;
volatile stc_gpio_pdora_field_t * const IO_PDORA_B __attribute__ ((section(".ioview"))) = (stc_gpio_pdora_field_t *)&PDORA;
volatile uint32_t * const IO_PDORB				__attribute__ ((section(".ioview"))) = &PDORB;
volatile stc_gpio_pdorb_field_t * const IO_PDORB_B __attribute__ ((section(".ioview"))) = (stc_gpio_pdorb_field_t *)&PDORB;
volatile uint32_t * const IO_PDORC				__attribute__ ((section(".ioview"))) = &PDORC;
volatile stc_gpio_pdorc_field_t * const IO_PDORC_B __attribute__ ((section(".ioview"))) = (stc_gpio_pdorc_field_t *)&PDORC;
volatile uint32_t * const IO_PDORD				__attribute__ ((section(".ioview"))) = &PDORD;
volatile stc_gpio_pdord_field_t * const IO_PDORD_B __attribute__ ((section(".ioview"))) = (stc_gpio_pdord_field_t *)&PDORD;
volatile uint32_t * const IO_PDORE				__attribute__ ((section(".ioview"))) = &PDORE;
volatile stc_gpio_pdore_field_t * const IO_PDORE_B __attribute__ ((section(".ioview"))) = (stc_gpio_pdore_field_t *)&PDORE;
volatile uint32_t * const IO_PDORF				__attribute__ ((section(".ioview"))) = &PDORF;
volatile stc_gpio_pdorf_field_t * const IO_PDORF_B __attribute__ ((section(".ioview"))) = (stc_gpio_pdorf_field_t *)&PDORF;
volatile uint32_t * const IO_ADE 				__attribute__ ((section(".ioview"))) = &ADE;
volatile uint32_t * const IO_SPSR				__attribute__ ((section(".ioview"))) = &SPSR;
volatile uint32_t * const IO_EPFR00 			__attribute__ ((section(".ioview"))) = &EPFR00;
volatile uint32_t * const IO_EPFR01 			__attribute__ ((section(".ioview"))) = &EPFR01;
volatile uint32_t * const IO_EPFR02 			__attribute__ ((section(".ioview"))) = &EPFR02;
volatile uint32_t * const IO_EPFR03 			__attribute__ ((section(".ioview"))) = &EPFR03;
volatile uint32_t * const IO_EPFR04 			__attribute__ ((section(".ioview"))) = &EPFR04;
volatile uint32_t * const IO_EPFR05 			__attribute__ ((section(".ioview"))) = &EPFR05;
volatile uint32_t * const IO_EPFR06 			__attribute__ ((section(".ioview"))) = &EPFR06;
volatile uint32_t * const IO_EPFR07 			__attribute__ ((section(".ioview"))) = &EPFR07;
volatile uint32_t * const IO_EPFR08 			__attribute__ ((section(".ioview"))) = &EPFR08;
volatile uint32_t * const IO_EPFR09 			__attribute__ ((section(".ioview"))) = &EPFR09;
volatile uint32_t * const IO_EPFR10 			__attribute__ ((section(".ioview"))) = &EPFR10;
volatile uint32_t * const IO_EPFR11 			__attribute__ ((section(".ioview"))) = &EPFR11;
volatile uint32_t * const IO_EPFR12 			__attribute__ ((section(".ioview"))) = &EPFR12;
volatile uint32_t * const IO_EPFR13 			__attribute__ ((section(".ioview"))) = &EPFR13;
volatile uint32_t * const IO_EPFR14 			__attribute__ ((section(".ioview"))) = &EPFR14;
volatile uint32_t * const IO_EPFR15 			__attribute__ ((section(".ioview"))) = &EPFR15;
volatile uint32_t * const IO_PZR0				__attribute__ ((section(".ioview"))) = &PZR0;
volatile uint32_t * const IO_PZR1				__attribute__ ((section(".ioview"))) = &PZR1;
volatile uint32_t * const IO_PZR2				__attribute__ ((section(".ioview"))) = &PZR2;
volatile uint32_t * const IO_PZR3				__attribute__ ((section(".ioview"))) = &PZR3;
volatile uint32_t * const IO_PZR4				__attribute__ ((section(".ioview"))) = &PZR4;
volatile uint32_t * const IO_PZR5				__attribute__ ((section(".ioview"))) = &PZR5;
volatile uint32_t * const IO_PZR6				__attribute__ ((section(".ioview"))) = &PZR6;
volatile uint32_t * const IO_PZR7				__attribute__ ((section(".ioview"))) = &PZR7;
volatile uint32_t * const IO_PZR8				__attribute__ ((section(".ioview"))) = &PZR8;
volatile uint32_t * const IO_PZR9				__attribute__ ((section(".ioview"))) = &PZR9;
volatile uint32_t * const IO_PZRA				__attribute__ ((section(".ioview"))) = &PZRA;
volatile uint32_t * const IO_PZRB				__attribute__ ((section(".ioview"))) = &PZRB;
volatile uint32_t * const IO_PZRC				__attribute__ ((section(".ioview"))) = &PZRC;
volatile uint32_t * const IO_PZRD				__attribute__ ((section(".ioview"))) = &PZRD;
volatile uint32_t * const IO_PZRE				__attribute__ ((section(".ioview"))) = &PZRE;
volatile uint32_t * const IO_PZRF				__attribute__ ((section(".ioview"))) = &PZRF;


/* IO_LVD */
volatile uint8_t  * const IO_LVD_CTL 			__attribute__ ((section(".ioview"))) = &LVD_CTL;
volatile uint8_t  * const IO_LVD_STR 			__attribute__ ((section(".ioview"))) = &LVD_STR;
volatile uint8_t  * const IO_LVD_CLR 			__attribute__ ((section(".ioview"))) = &LVD_CLR;
volatile uint32_t * const IO_LVD_RLR 			__attribute__ ((section(".ioview"))) = &LVD_RLR;
volatile uint8_t  * const IO_LVD_STR2			__attribute__ ((section(".ioview"))) = &LVD_STR2;


/* IO_DS_Mode */  
volatile uint8_t  * const IO_DSM_PMD_CTL		__attribute__ ((section(".ioview"))) = &DSM_PMD_CTL;
volatile uint8_t  * const IO_DSM_WRFSR			__attribute__ ((section(".ioview"))) = &DSM_WRFSR;
volatile uint8_t  * const IO_DSM_WIFSR			__attribute__ ((section(".ioview"))) = &DSM_WIFSR;
volatile uint8_t  * const IO_DSM_WIER			__attribute__ ((section(".ioview"))) = &DSM_WIER;
volatile uint8_t  * const IO_DSM_WILVR			__attribute__ ((section(".ioview"))) = &DSM_WILVR;
volatile uint8_t  * const IO_DSM_BUR01			__attribute__ ((section(".ioview"))) = &DSM_BUR01;
volatile uint8_t  * const IO_DSM_BUR02			__attribute__ ((section(".ioview"))) = &DSM_BUR02;
volatile uint8_t  * const IO_DSM_BUR03			__attribute__ ((section(".ioview"))) = &DSM_BUR03;
volatile uint8_t  * const IO_DSM_BUR04			__attribute__ ((section(".ioview"))) = &DSM_BUR04;
volatile uint8_t  * const IO_DSM_BUR05			__attribute__ ((section(".ioview"))) = &DSM_BUR05;
volatile uint8_t  * const IO_DSM_BUR06			__attribute__ ((section(".ioview"))) = &DSM_BUR06;
volatile uint8_t  * const IO_DSM_BUR07			__attribute__ ((section(".ioview"))) = &DSM_BUR07;
volatile uint8_t  * const IO_DSM_BUR08			__attribute__ ((section(".ioview"))) = &DSM_BUR08;
volatile uint8_t  * const IO_DSM_BUR09			__attribute__ ((section(".ioview"))) = &DSM_BUR09;
volatile uint8_t  * const IO_DSM_BUR10			__attribute__ ((section(".ioview"))) = &DSM_BUR10;
volatile uint8_t  * const IO_DSM_BUR11			__attribute__ ((section(".ioview"))) = &DSM_BUR11;
volatile uint8_t  * const IO_DSM_BUR12			__attribute__ ((section(".ioview"))) = &DSM_BUR12;
volatile uint8_t  * const IO_DSM_BUR13			__attribute__ ((section(".ioview"))) = &DSM_BUR13;
volatile uint8_t  * const IO_DSM_BUR14			__attribute__ ((section(".ioview"))) = &DSM_BUR14;
volatile uint8_t  * const IO_DSM_BUR15			__attribute__ ((section(".ioview"))) = &DSM_BUR15;
volatile uint8_t  * const IO_DSM_BUR16			__attribute__ ((section(".ioview"))) = &DSM_BUR16;


/* IO_USB Clock */
volatile uint8_t  * const IO_USBCLK_UCCR		__attribute__ ((section(".ioview"))) = &USBCLK_UCCR;
volatile uint8_t  * const IO_USBCLK_UPCR1 		__attribute__ ((section(".ioview"))) = &USBCLK_UPCR1;
volatile uint8_t  * const IO_USBCLK_UPCR2 		__attribute__ ((section(".ioview"))) = &USBCLK_UPCR2;
volatile uint8_t  * const IO_USBCLK_UPCR3 		__attribute__ ((section(".ioview"))) = &USBCLK_UPCR3;
volatile uint8_t  * const IO_USBCLK_UPCR4 		__attribute__ ((section(".ioview"))) = &USBCLK_UPCR4;
volatile uint8_t  * const IO_USBCLK_UP_STR		__attribute__ ((section(".ioview"))) = &USBCLK_UP_STR;
volatile uint8_t  * const IO_USBCLK_UPINT_ENR	__attribute__ ((section(".ioview"))) = &USBCLK_UPINT_ENR;
volatile uint8_t  * const IO_USBCLK_UPINT_CLR	__attribute__ ((section(".ioview"))) = &USBCLK_UPINT_CLR;
volatile uint8_t  * const IO_USBCLK_UPINT_STR	__attribute__ ((section(".ioview"))) = &USBCLK_UPINT_STR;
volatile uint8_t  * const IO_USBCLK_UPCR5 		__attribute__ ((section(".ioview"))) = &USBCLK_UPCR5;
volatile uint8_t  * const IO_USBCLK_UPCR6 		__attribute__ ((section(".ioview"))) = &USBCLK_UPCR6;
volatile uint8_t  * const IO_USBCLK_UPCR7 		__attribute__ ((section(".ioview"))) = &USBCLK_UPCR7;
volatile uint8_t  * const IO_USBCLK_USBEN 		__attribute__ ((section(".ioview"))) = &USBCLK_USBEN;
volatile uint8_t  * const IO_USBCLK_USBEN1		__attribute__ ((section(".ioview"))) = &USBCLK_USBEN1;


/* IO_CAN Prescaler */   
volatile uint8_t  * const IO_CANPRE 				__attribute__ ((section(".ioview"))) = &CANPRE;


/* IO_MFS ch0-3 */
volatile uint8_t  * const IO_MFS0_SMR			__attribute__ ((section(".ioview"))) = &MFS0_SMR;
volatile uint8_t  * const IO_MFS0_SCR			__attribute__ ((section(".ioview"))) = &MFS0_SCR;
volatile uint8_t  * const IO_MFS0_IBCR 			__attribute__ ((section(".ioview"))) = &MFS0_IBCR;
volatile uint8_t  * const IO_MFS0_ESCR 			__attribute__ ((section(".ioview"))) = &MFS0_ESCR;
volatile uint8_t  * const IO_MFS0_IBSR 			__attribute__ ((section(".ioview"))) = &MFS0_IBSR;
volatile uint8_t  * const IO_MFS0_SSR			__attribute__ ((section(".ioview"))) = &MFS0_SSR;
volatile uint16_t * const IO_MFS0_TDR			__attribute__ ((section(".ioview"))) = &MFS0_TDR;
volatile uint16_t * const IO_MFS0_RDR			__attribute__ ((section(".ioview"))) = &MFS0_RDR;
volatile uint16_t * const IO_MFS0_BGR			__attribute__ ((section(".ioview"))) = &MFS0_BGR;
volatile uint8_t  * const IO_MFS0_ISBA 			__attribute__ ((section(".ioview"))) = &MFS0_ISBA;
volatile uint8_t  * const IO_MFS0_ISMK 			__attribute__ ((section(".ioview"))) = &MFS0_ISMK;
 
volatile uint8_t  * const IO_MFS1_SMR			__attribute__ ((section(".ioview"))) = &MFS1_SMR;
volatile uint8_t  * const IO_MFS1_SCR			__attribute__ ((section(".ioview"))) = &MFS1_SCR;
volatile uint8_t  * const IO_MFS1_IBCR 			__attribute__ ((section(".ioview"))) = &MFS1_IBCR;
volatile uint8_t  * const IO_MFS1_ESCR 			__attribute__ ((section(".ioview"))) = &MFS1_ESCR;
volatile uint8_t  * const IO_MFS1_IBSR 			__attribute__ ((section(".ioview"))) = &MFS1_IBSR;
volatile uint8_t  * const IO_MFS1_SSR			__attribute__ ((section(".ioview"))) = &MFS1_SSR;
volatile uint16_t * const IO_MFS1_TDR			__attribute__ ((section(".ioview"))) = &MFS1_TDR;
volatile uint16_t * const IO_MFS1_RDR			__attribute__ ((section(".ioview"))) = &MFS1_RDR;
volatile uint16_t * const IO_MFS1_BGR			__attribute__ ((section(".ioview"))) = &MFS1_BGR;
volatile uint8_t  * const IO_MFS1_ISBA 			__attribute__ ((section(".ioview"))) = &MFS1_ISBA;
volatile uint8_t  * const IO_MFS1_ISMK 			__attribute__ ((section(".ioview"))) = &MFS1_ISMK;
 
volatile uint8_t  * const IO_MFS2_SMR			__attribute__ ((section(".ioview"))) = &MFS2_SMR;
volatile uint8_t  * const IO_MFS2_SCR			__attribute__ ((section(".ioview"))) = &MFS2_SCR;
volatile uint8_t  * const IO_MFS2_IBCR 			__attribute__ ((section(".ioview"))) = &MFS2_IBCR;
volatile uint8_t  * const IO_MFS2_ESCR 			__attribute__ ((section(".ioview"))) = &MFS2_ESCR;
volatile uint8_t  * const IO_MFS2_IBSR 			__attribute__ ((section(".ioview"))) = &MFS2_IBSR;
volatile uint8_t  * const IO_MFS2_SSR			__attribute__ ((section(".ioview"))) = &MFS2_SSR;
volatile uint16_t * const IO_MFS2_TDR			__attribute__ ((section(".ioview"))) = &MFS2_TDR;
volatile uint16_t * const IO_MFS2_RDR			__attribute__ ((section(".ioview"))) = &MFS2_RDR;
volatile uint16_t * const IO_MFS2_BGR			__attribute__ ((section(".ioview"))) = &MFS2_BGR;
volatile uint8_t  * const IO_MFS2_ISBA 			__attribute__ ((section(".ioview"))) = &MFS2_ISBA;
volatile uint8_t  * const IO_MFS2_ISMK 			__attribute__ ((section(".ioview"))) = &MFS2_ISMK;
 
volatile uint8_t  * const IO_MFS3_SMR			__attribute__ ((section(".ioview"))) = &MFS3_SMR;
volatile uint8_t  * const IO_MFS3_SCR			__attribute__ ((section(".ioview"))) = &MFS3_SCR;
volatile uint8_t  * const IO_MFS3_IBCR			__attribute__ ((section(".ioview"))) = &MFS3_IBCR;
volatile uint8_t  * const IO_MFS3_ESCR			__attribute__ ((section(".ioview"))) = &MFS3_ESCR;
volatile uint8_t  * const IO_MFS3_IBSR			__attribute__ ((section(".ioview"))) = &MFS3_IBSR;
volatile uint8_t  * const IO_MFS3_SSR			__attribute__ ((section(".ioview"))) = &MFS3_SSR;
volatile uint16_t * const IO_MFS3_TDR			__attribute__ ((section(".ioview"))) = &MFS3_TDR;
volatile uint16_t * const IO_MFS3_RDR			__attribute__ ((section(".ioview"))) = &MFS3_RDR;
volatile uint16_t * const IO_MFS3_BGR			__attribute__ ((section(".ioview"))) = &MFS3_BGR;
volatile uint8_t  * const IO_MFS3_ISBA			__attribute__ ((section(".ioview"))) = &MFS3_ISBA;
volatile uint8_t  * const IO_MFS3_ISMK			__attribute__ ((section(".ioview"))) = &MFS3_ISMK;


/* IO_MFS ch4-7 */
volatile uint8_t  * const IO_MFS4_SMR			__attribute__ ((section(".ioview"))) = &MFS4_SMR;
volatile uint8_t  * const IO_MFS4_SCR			__attribute__ ((section(".ioview"))) = &MFS4_SCR;
volatile uint8_t  * const IO_MFS4_IBCR			__attribute__ ((section(".ioview"))) = &MFS4_IBCR;
volatile uint8_t  * const IO_MFS4_ESCR			__attribute__ ((section(".ioview"))) = &MFS4_ESCR;
volatile uint8_t  * const IO_MFS4_IBSR			__attribute__ ((section(".ioview"))) = &MFS4_IBSR;
volatile uint8_t  * const IO_MFS4_SSR			__attribute__ ((section(".ioview"))) = &MFS4_SSR;
volatile uint16_t * const IO_MFS4_TDR			__attribute__ ((section(".ioview"))) = &MFS4_TDR;
volatile uint16_t * const IO_MFS4_RDR			__attribute__ ((section(".ioview"))) = &MFS4_RDR;
volatile uint16_t * const IO_MFS4_BGR			__attribute__ ((section(".ioview"))) = &MFS4_BGR;
volatile uint8_t  * const IO_MFS4_ISBA			__attribute__ ((section(".ioview"))) = &MFS4_ISBA;
volatile uint8_t  * const IO_MFS4_ISMK			__attribute__ ((section(".ioview"))) = &MFS4_ISMK;
volatile uint16_t * const IO_MFS4_FCR			__attribute__ ((section(".ioview"))) = &MFS4_FCR;
volatile uint8_t  * const IO_MFS4_FCR0			__attribute__ ((section(".ioview"))) = &MFS4_FCR0;
volatile uint8_t  * const IO_MFS4_FCR1			__attribute__ ((section(".ioview"))) = &MFS4_FCR1;
volatile uint16_t * const IO_MFS4_FBYTE 		__attribute__ ((section(".ioview"))) = &MFS4_FBYTE;
volatile uint8_t  * const IO_MFS4_FBYTE1		__attribute__ ((section(".ioview"))) = &MFS4_FBYTE1;
volatile uint8_t  * const IO_MFS4_FBYTE2		__attribute__ ((section(".ioview"))) = &MFS4_FBYTE2;

volatile uint8_t  * const IO_MFS5_SMR			__attribute__ ((section(".ioview"))) = &MFS5_SMR;
volatile uint8_t  * const IO_MFS5_SCR			__attribute__ ((section(".ioview"))) = &MFS5_SCR;
volatile uint8_t  * const IO_MFS5_IBCR			__attribute__ ((section(".ioview"))) = &MFS5_IBCR;
volatile uint8_t  * const IO_MFS5_ESCR			__attribute__ ((section(".ioview"))) = &MFS5_ESCR;
volatile uint8_t  * const IO_MFS5_IBSR			__attribute__ ((section(".ioview"))) = &MFS5_IBSR;
volatile uint8_t  * const IO_MFS5_SSR			__attribute__ ((section(".ioview"))) = &MFS5_SSR;
volatile uint16_t * const IO_MFS5_TDR			__attribute__ ((section(".ioview"))) = &MFS5_TDR;
volatile uint16_t * const IO_MFS5_RDR			__attribute__ ((section(".ioview"))) = &MFS5_RDR;
volatile uint16_t * const IO_MFS5_BGR			__attribute__ ((section(".ioview"))) = &MFS5_BGR;
volatile uint8_t  * const IO_MFS5_ISBA			__attribute__ ((section(".ioview"))) = &MFS5_ISBA;
volatile uint8_t  * const IO_MFS5_ISMK			__attribute__ ((section(".ioview"))) = &MFS5_ISMK;
volatile uint16_t * const IO_MFS5_FCR			__attribute__ ((section(".ioview"))) = &MFS5_FCR;
volatile uint8_t  * const IO_MFS5_FCR0			__attribute__ ((section(".ioview"))) = &MFS5_FCR0;
volatile uint8_t  * const IO_MFS5_FCR1			__attribute__ ((section(".ioview"))) = &MFS5_FCR1;
volatile uint16_t * const IO_MFS5_FBYTE 		__attribute__ ((section(".ioview"))) = &MFS5_FBYTE;
volatile uint8_t  * const IO_MFS5_FBYTE1		__attribute__ ((section(".ioview"))) = &MFS5_FBYTE1;
volatile uint8_t  * const IO_MFS5_FBYTE2		__attribute__ ((section(".ioview"))) = &MFS5_FBYTE2;

volatile uint8_t  * const IO_MFS6_SMR			__attribute__ ((section(".ioview"))) = &MFS6_SMR;
volatile uint8_t  * const IO_MFS6_SCR			__attribute__ ((section(".ioview"))) = &MFS6_SCR;
volatile uint8_t  * const IO_MFS6_IBCR			__attribute__ ((section(".ioview"))) = &MFS6_IBCR;
volatile uint8_t  * const IO_MFS6_ESCR			__attribute__ ((section(".ioview"))) = &MFS6_ESCR;
volatile uint8_t  * const IO_MFS6_IBSR			__attribute__ ((section(".ioview"))) = &MFS6_IBSR;
volatile uint8_t  * const IO_MFS6_SSR			__attribute__ ((section(".ioview"))) = &MFS6_SSR;
volatile uint16_t * const IO_MFS6_TDR			__attribute__ ((section(".ioview"))) = &MFS6_TDR;
volatile uint16_t * const IO_MFS6_RDR			__attribute__ ((section(".ioview"))) = &MFS6_RDR;
volatile uint16_t * const IO_MFS6_BGR			__attribute__ ((section(".ioview"))) = &MFS6_BGR;
volatile uint8_t  * const IO_MFS6_ISBA			__attribute__ ((section(".ioview"))) = &MFS6_ISBA;
volatile uint8_t  * const IO_MFS6_ISMK			__attribute__ ((section(".ioview"))) = &MFS6_ISMK;
volatile uint16_t * const IO_MFS6_FCR			__attribute__ ((section(".ioview"))) = &MFS6_FCR;
volatile uint8_t  * const IO_MFS6_FCR0			__attribute__ ((section(".ioview"))) = &MFS6_FCR0;
volatile uint8_t  * const IO_MFS6_FCR1			__attribute__ ((section(".ioview"))) = &MFS6_FCR1;
volatile uint16_t * const IO_MFS6_FBYTE 		__attribute__ ((section(".ioview"))) = &MFS6_FBYTE;
volatile uint8_t  * const IO_MFS6_FBYTE1		__attribute__ ((section(".ioview"))) = &MFS6_FBYTE1;
volatile uint8_t  * const IO_MFS6_FBYTE2		__attribute__ ((section(".ioview"))) = &MFS6_FBYTE2;

volatile uint8_t  * const IO_MFS7_SMR			__attribute__ ((section(".ioview"))) = &MFS7_SMR;
volatile uint8_t  * const IO_MFS7_SCR			__attribute__ ((section(".ioview"))) = &MFS7_SCR;
volatile uint8_t  * const IO_MFS7_IBCR			__attribute__ ((section(".ioview"))) = &MFS7_IBCR;
volatile uint8_t  * const IO_MFS7_ESCR			__attribute__ ((section(".ioview"))) = &MFS7_ESCR;
volatile uint8_t  * const IO_MFS7_IBSR			__attribute__ ((section(".ioview"))) = &MFS7_IBSR;
volatile uint8_t  * const IO_MFS7_SSR			__attribute__ ((section(".ioview"))) = &MFS7_SSR;
volatile uint16_t * const IO_MFS7_TDR			__attribute__ ((section(".ioview"))) = &MFS7_TDR;
volatile uint16_t * const IO_MFS7_RDR			__attribute__ ((section(".ioview"))) = &MFS7_RDR;
volatile uint16_t * const IO_MFS7_BGR			__attribute__ ((section(".ioview"))) = &MFS7_BGR;
volatile uint8_t  * const IO_MFS7_ISBA			__attribute__ ((section(".ioview"))) = &MFS7_ISBA;
volatile uint8_t  * const IO_MFS7_ISMK			__attribute__ ((section(".ioview"))) = &MFS7_ISMK;
volatile uint16_t * const IO_MFS7_FCR			__attribute__ ((section(".ioview"))) = &MFS7_FCR;
volatile uint8_t  * const IO_MFS7_FCR0			__attribute__ ((section(".ioview"))) = &MFS7_FCR0;
volatile uint8_t  * const IO_MFS7_FCR1			__attribute__ ((section(".ioview"))) = &MFS7_FCR1;
volatile uint16_t * const IO_MFS7_FBYTE 		__attribute__ ((section(".ioview"))) = &MFS7_FBYTE;
volatile uint8_t  * const IO_MFS7_FBYTE1		__attribute__ ((section(".ioview"))) = &MFS7_FBYTE1;
volatile uint8_t  * const IO_MFS7_FBYTE2		__attribute__ ((section(".ioview"))) = &MFS7_FBYTE2;

/* IO_MSS Noise Filter */
volatile uint32_t * const IO_MFS4_I2CDNF		__attribute__ ((section(".ioview"))) = &MFS4_I2CDNF;

/* IO_CRC */
volatile uint8_t  * const IO_CRC_CRCCR			__attribute__ ((section(".ioview"))) = &CRC_CRCCR;
volatile uint32_t * const IO_CRC_CRCINIT		__attribute__ ((section(".ioview"))) = &CRC_CRCINIT;
volatile uint32_t * const IO_CRC_CRCIN			__attribute__ ((section(".ioview"))) = &CRC_CRCIN;
volatile uint32_t * const IO_CRC_CRCR			__attribute__ ((section(".ioview"))) = &CRC_CRCR;

/* IO_Watch Counter */   
volatile uint8_t  * const IO_WC_WCRD 			__attribute__ ((section(".ioview"))) = &WC_WCRD;
volatile uint8_t  * const IO_WC_WCRL 			__attribute__ ((section(".ioview"))) = &WC_WCRL;
volatile uint8_t  * const IO_WC_WCCR 			__attribute__ ((section(".ioview"))) = &WC_WCCR;
volatile uint16_t * const IO_WC_CLK_SEL 		__attribute__ ((section(".ioview"))) = &WC_CLK_SEL;
volatile uint8_t  * const IO_WC_CLK_EN			__attribute__ ((section(".ioview"))) = &WC_CLK_EN;

/* IO_RTC */
volatile uint32_t * const IO_RTC_WTCR1			__attribute__ ((section(".ioview"))) = &RTC_WTCR1;
volatile uint32_t * const IO_RTC_WTCR2			__attribute__ ((section(".ioview"))) = &RTC_WTCR2;
volatile uint32_t * const IO_RTC_WTBR			__attribute__ ((section(".ioview"))) = &RTC_WTBR;
volatile uint8_t  * const IO_RTC_WTSR			__attribute__ ((section(".ioview"))) = &RTC_WTSR;
volatile uint8_t  * const IO_RTC_WTMIR			__attribute__ ((section(".ioview"))) = &RTC_WTMIR;
volatile uint8_t  * const IO_RTC_WTHR			__attribute__ ((section(".ioview"))) = &RTC_WTHR;
volatile uint8_t  * const IO_RTC_WTDR			__attribute__ ((section(".ioview"))) = &RTC_WTDR;
volatile uint8_t  * const IO_RTC_WTDW			__attribute__ ((section(".ioview"))) = &RTC_WTDW;
volatile uint8_t  * const IO_RTC_WTMOR			__attribute__ ((section(".ioview"))) = &RTC_WTMOR;
volatile uint8_t  * const IO_RTC_WTYR			__attribute__ ((section(".ioview"))) = &RTC_WTYR;
volatile uint8_t  * const IO_RTC_ALMIR			__attribute__ ((section(".ioview"))) = &RTC_ALMIR;
volatile uint8_t  * const IO_RTC_ALHR			__attribute__ ((section(".ioview"))) = &RTC_ALHR;
volatile uint8_t  * const IO_RTC_ALDR			__attribute__ ((section(".ioview"))) = &RTC_ALDR;
volatile uint8_t  * const IO_RTC_ALMOR			__attribute__ ((section(".ioview"))) = &RTC_ALMOR;
volatile uint8_t  * const IO_RTC_ALYR			__attribute__ ((section(".ioview"))) = &RTC_ALYR;
volatile uint32_t * const IO_RTC_WTTR			__attribute__ ((section(".ioview"))) = &RTC_WTTR;
volatile uint8_t  * const IO_RTC_WTCLKS			__attribute__ ((section(".ioview"))) = &RTC_WTCLKS;
volatile uint8_t  * const IO_RTC_WTCLKM			__attribute__ ((section(".ioview"))) = &RTC_WTCLKM;
volatile uint8_t  * const IO_RTC_WTCAL			__attribute__ ((section(".ioview"))) = &RTC_WTCAL;
volatile uint8_t  * const IO_RTC_WTCALEN 		__attribute__ ((section(".ioview"))) = &RTC_WTCALEN;
volatile uint8_t  * const IO_RTC_WTDIV			__attribute__ ((section(".ioview"))) = &RTC_WTDIV;
volatile uint8_t  * const IO_RTC_WTDIVEN		__attribute__ ((section(".ioview"))) = &RTC_WTDIVEN;

/* IO_EXT-Bus I/F */
volatile uint32_t * const IO_EXTBUS_MODE0 		__attribute__ ((section(".ioview"))) = &EXTBUS_MODE0;
volatile uint32_t * const IO_EXTBUS_MODE1 		__attribute__ ((section(".ioview"))) = &EXTBUS_MODE1;
volatile uint32_t * const IO_EXTBUS_MODE2 		__attribute__ ((section(".ioview"))) = &EXTBUS_MODE2;
volatile uint32_t * const IO_EXTBUS_MODE3 		__attribute__ ((section(".ioview"))) = &EXTBUS_MODE3;
volatile uint32_t * const IO_EXTBUS_MODE4 		__attribute__ ((section(".ioview"))) = &EXTBUS_MODE4;
volatile uint32_t * const IO_EXTBUS_MODE5 		__attribute__ ((section(".ioview"))) = &EXTBUS_MODE5;
volatile uint32_t * const IO_EXTBUS_MODE6 		__attribute__ ((section(".ioview"))) = &EXTBUS_MODE6;
volatile uint32_t * const IO_EXTBUS_MODE7 		__attribute__ ((section(".ioview"))) = &EXTBUS_MODE7;
volatile uint32_t * const IO_EXTBUS_TIM0		__attribute__ ((section(".ioview"))) = &EXTBUS_TIM0;
volatile uint32_t * const IO_EXTBUS_TIM1		__attribute__ ((section(".ioview"))) = &EXTBUS_TIM1;
volatile uint32_t * const IO_EXTBUS_TIM2		__attribute__ ((section(".ioview"))) = &EXTBUS_TIM2;
volatile uint32_t * const IO_EXTBUS_TIM3		__attribute__ ((section(".ioview"))) = &EXTBUS_TIM3;
volatile uint32_t * const IO_EXTBUS_TIM4		__attribute__ ((section(".ioview"))) = &EXTBUS_TIM4;
volatile uint32_t * const IO_EXTBUS_TIM5		__attribute__ ((section(".ioview"))) = &EXTBUS_TIM5;
volatile uint32_t * const IO_EXTBUS_TIM6		__attribute__ ((section(".ioview"))) = &EXTBUS_TIM6;
volatile uint32_t * const IO_EXTBUS_TIM7		__attribute__ ((section(".ioview"))) = &EXTBUS_TIM7;
volatile uint32_t * const IO_EXTBUS_AREA0 		__attribute__ ((section(".ioview"))) = &EXTBUS_AREA0;
volatile uint32_t * const IO_EXTBUS_AREA1 		__attribute__ ((section(".ioview"))) = &EXTBUS_AREA1;
volatile uint32_t * const IO_EXTBUS_AREA2 		__attribute__ ((section(".ioview"))) = &EXTBUS_AREA2;
volatile uint32_t * const IO_EXTBUS_AREA3 		__attribute__ ((section(".ioview"))) = &EXTBUS_AREA3;
volatile uint32_t * const IO_EXTBUS_AREA4 		__attribute__ ((section(".ioview"))) = &EXTBUS_AREA4;
volatile uint32_t * const IO_EXTBUS_AREA5 		__attribute__ ((section(".ioview"))) = &EXTBUS_AREA5;
volatile uint32_t * const IO_EXTBUS_AREA6 		__attribute__ ((section(".ioview"))) = &EXTBUS_AREA6;
volatile uint32_t * const IO_EXTBUS_AREA7 		__attribute__ ((section(".ioview"))) = &EXTBUS_AREA7;
volatile uint32_t * const IO_EXTBUS_ATIM0 		__attribute__ ((section(".ioview"))) = &EXTBUS_ATIM0;
volatile uint32_t * const IO_EXTBUS_ATIM1 		__attribute__ ((section(".ioview"))) = &EXTBUS_ATIM1;
volatile uint32_t * const IO_EXTBUS_ATIM2 		__attribute__ ((section(".ioview"))) = &EXTBUS_ATIM2;
volatile uint32_t * const IO_EXTBUS_ATIM3 		__attribute__ ((section(".ioview"))) = &EXTBUS_ATIM3;
volatile uint32_t * const IO_EXTBUS_ATIM4 		__attribute__ ((section(".ioview"))) = &EXTBUS_ATIM4;
volatile uint32_t * const IO_EXTBUS_ATIM5 		__attribute__ ((section(".ioview"))) = &EXTBUS_ATIM5;
volatile uint32_t * const IO_EXTBUS_ATIM6 		__attribute__ ((section(".ioview"))) = &EXTBUS_ATIM6;
volatile uint32_t * const IO_EXTBUS_ATIM7 		__attribute__ ((section(".ioview"))) = &EXTBUS_ATIM7;
volatile uint32_t * const IO_EXTBUS_DCLKR 		__attribute__ ((section(".ioview"))) = &EXTBUS_DCLKR;

/* IO_USB ch0-1 */
volatile uint8_t  * const IO_USB0_HCNT0 		__attribute__ ((section(".ioview"))) = &USB0_HCNT0;
volatile uint8_t  * const IO_USB0_HCNT1 		__attribute__ ((section(".ioview"))) = &USB0_HCNT1;
volatile uint8_t  * const IO_USB0_HIRQ			__attribute__ ((section(".ioview"))) = &USB0_HIRQ;
volatile uint8_t  * const IO_USB0_HERR			__attribute__ ((section(".ioview"))) = &USB0_HERR;
volatile uint8_t  * const IO_USB0_HSTATE		__attribute__ ((section(".ioview"))) = &USB0_HSTATE;
volatile uint8_t  * const IO_USB0_HFCOMP		__attribute__ ((section(".ioview"))) = &USB0_HFCOMP;
volatile uint16_t * const IO_USB0_HRTIMER10		__attribute__ ((section(".ioview"))) = &USB0_HRTIMER10;
volatile uint8_t  * const IO_USB0_HRTIMER2 		__attribute__ ((section(".ioview"))) = &USB0_HRTIMER2;
volatile uint8_t  * const IO_USB0_HADR			__attribute__ ((section(".ioview"))) = &USB0_HADR;
volatile uint16_t * const IO_USB0_HEOF10		__attribute__ ((section(".ioview"))) = &USB0_HEOF10;
volatile uint16_t * const IO_USB0_HFRAME10 		__attribute__ ((section(".ioview"))) = &USB0_HFRAME10;
volatile uint8_t  * const IO_USB0_HTOKEN		__attribute__ ((section(".ioview"))) = &USB0_HTOKEN;
volatile uint16_t * const IO_USB0_UDCC			__attribute__ ((section(".ioview"))) = &USB0_UDCC;
volatile uint16_t * const IO_USB0_EP0C			__attribute__ ((section(".ioview"))) = &USB0_EP0C;
volatile uint16_t * const IO_USB0_EP1C			__attribute__ ((section(".ioview"))) = &USB0_EP1C;
volatile uint16_t * const IO_USB0_EP2C			__attribute__ ((section(".ioview"))) = &USB0_EP2C;
volatile uint16_t * const IO_USB0_EP3C			__attribute__ ((section(".ioview"))) = &USB0_EP3C;
volatile uint16_t * const IO_USB0_EP4C			__attribute__ ((section(".ioview"))) = &USB0_EP4C;
volatile uint16_t * const IO_USB0_EP5C			__attribute__ ((section(".ioview"))) = &USB0_EP5C;
volatile uint16_t * const IO_USB0_TMSP			__attribute__ ((section(".ioview"))) = &USB0_TMSP;
volatile uint8_t  * const IO_USB0_UDCS			__attribute__ ((section(".ioview"))) = &USB0_UDCS;
volatile uint8_t  * const IO_USB0_UDCIE 		__attribute__ ((section(".ioview"))) = &USB0_UDCIE;
volatile uint16_t * const IO_USB0_EP0IS 		__attribute__ ((section(".ioview"))) = &USB0_EP0IS;
volatile uint16_t * const IO_USB0_EP0OS 		__attribute__ ((section(".ioview"))) = &USB0_EP0OS;
volatile uint16_t * const IO_USB0_EP1S			__attribute__ ((section(".ioview"))) = &USB0_EP1S;
volatile uint16_t * const IO_USB0_EP2S			__attribute__ ((section(".ioview"))) = &USB0_EP2S;
volatile uint16_t * const IO_USB0_EP3S			__attribute__ ((section(".ioview"))) = &USB0_EP3S;
volatile uint16_t * const IO_USB0_EP4S			__attribute__ ((section(".ioview"))) = &USB0_EP4S;
volatile uint16_t * const IO_USB0_EP5S			__attribute__ ((section(".ioview"))) = &USB0_EP5S;
volatile uint8_t  * const IO_USB0_EP0DTL		__attribute__ ((section(".ioview"))) = &USB0_EP0DTL;
volatile uint8_t  * const IO_USB0_EP0DTH		__attribute__ ((section(".ioview"))) = &USB0_EP0DTH;
volatile uint8_t  * const IO_USB0_EP1DTL		__attribute__ ((section(".ioview"))) = &USB0_EP1DTL;
volatile uint8_t  * const IO_USB0_EP1DTH		__attribute__ ((section(".ioview"))) = &USB0_EP1DTH;
volatile uint8_t  * const IO_USB0_EP2DTL		__attribute__ ((section(".ioview"))) = &USB0_EP2DTL;
volatile uint8_t  * const IO_USB0_EP2DTH		__attribute__ ((section(".ioview"))) = &USB0_EP2DTH;
volatile uint8_t  * const IO_USB0_EP3DTL		__attribute__ ((section(".ioview"))) = &USB0_EP3DTL;
volatile uint8_t  * const IO_USB0_EP3DTH		__attribute__ ((section(".ioview"))) = &USB0_EP3DTH;
volatile uint8_t  * const IO_USB0_EP4DTL		__attribute__ ((section(".ioview"))) = &USB0_EP4DTL;
volatile uint8_t  * const IO_USB0_EP4DTH		__attribute__ ((section(".ioview"))) = &USB0_EP4DTH;
volatile uint8_t  * const IO_USB0_EP5DTL		__attribute__ ((section(".ioview"))) = &USB0_EP5DTL;
volatile uint8_t  * const IO_USB0_EP5DTH		__attribute__ ((section(".ioview"))) = &USB0_EP5DTH;

volatile uint8_t  * const IO_USB1_HCNT0 		__attribute__ ((section(".ioview"))) = &USB1_HCNT0;
volatile uint8_t  * const IO_USB1_HCNT1 		__attribute__ ((section(".ioview"))) = &USB1_HCNT1;
volatile uint8_t  * const IO_USB1_HIRQ			__attribute__ ((section(".ioview"))) = &USB1_HIRQ;
volatile uint8_t  * const IO_USB1_HERR			__attribute__ ((section(".ioview"))) = &USB1_HERR;
volatile uint8_t  * const IO_USB1_HSTATE		__attribute__ ((section(".ioview"))) = &USB1_HSTATE;
volatile uint8_t  * const IO_USB1_HFCOMP		__attribute__ ((section(".ioview"))) = &USB1_HFCOMP;
volatile uint16_t * const IO_USB1_HRTIMER10		__attribute__ ((section(".ioview"))) = &USB1_HRTIMER10;
volatile uint8_t  * const IO_USB1_HRTIMER2 		__attribute__ ((section(".ioview"))) = &USB1_HRTIMER2;
volatile uint8_t  * const IO_USB1_HADR			__attribute__ ((section(".ioview"))) = &USB1_HADR;
volatile uint16_t * const IO_USB1_HEOF10		__attribute__ ((section(".ioview"))) = &USB1_HEOF10;
volatile uint16_t * const IO_USB1_HFRAME10 		__attribute__ ((section(".ioview"))) = &USB1_HFRAME10;
volatile uint8_t  * const IO_USB1_HTOKEN		__attribute__ ((section(".ioview"))) = &USB1_HTOKEN;
volatile uint16_t * const IO_USB1_UDCC			__attribute__ ((section(".ioview"))) = &USB1_UDCC;
volatile uint16_t * const IO_USB1_EP0C			__attribute__ ((section(".ioview"))) = &USB1_EP0C;
volatile uint16_t * const IO_USB1_EP1C			__attribute__ ((section(".ioview"))) = &USB1_EP1C;
volatile uint16_t * const IO_USB1_EP2C			__attribute__ ((section(".ioview"))) = &USB1_EP2C;
volatile uint16_t * const IO_USB1_EP3C			__attribute__ ((section(".ioview"))) = &USB1_EP3C;
volatile uint16_t * const IO_USB1_EP4C			__attribute__ ((section(".ioview"))) = &USB1_EP4C;
volatile uint16_t * const IO_USB1_EP5C			__attribute__ ((section(".ioview"))) = &USB1_EP5C;
volatile uint16_t * const IO_USB1_TMSP			__attribute__ ((section(".ioview"))) = &USB1_TMSP;
volatile uint8_t  * const IO_USB1_UDCS			__attribute__ ((section(".ioview"))) = &USB1_UDCS;
volatile uint8_t  * const IO_USB1_UDCIE 		__attribute__ ((section(".ioview"))) = &USB1_UDCIE;
volatile uint16_t * const IO_USB1_EP0IS 		__attribute__ ((section(".ioview"))) = &USB1_EP0IS;
volatile uint16_t * const IO_USB1_EP0OS 		__attribute__ ((section(".ioview"))) = &USB1_EP0OS;
volatile uint16_t * const IO_USB1_EP1S			__attribute__ ((section(".ioview"))) = &USB1_EP1S;
volatile uint16_t * const IO_USB1_EP2S			__attribute__ ((section(".ioview"))) = &USB1_EP2S;
volatile uint16_t * const IO_USB1_EP3S			__attribute__ ((section(".ioview"))) = &USB1_EP3S;
volatile uint16_t * const IO_USB1_EP4S			__attribute__ ((section(".ioview"))) = &USB1_EP4S;
volatile uint16_t * const IO_USB1_EP5S			__attribute__ ((section(".ioview"))) = &USB1_EP5S;
volatile uint8_t  * const IO_USB1_EP0DTL		__attribute__ ((section(".ioview"))) = &USB1_EP0DTL;
volatile uint8_t  * const IO_USB1_EP0DTH		__attribute__ ((section(".ioview"))) = &USB1_EP0DTH;
volatile uint8_t  * const IO_USB1_EP1DTL		__attribute__ ((section(".ioview"))) = &USB1_EP1DTL;
volatile uint8_t  * const IO_USB1_EP1DTH		__attribute__ ((section(".ioview"))) = &USB1_EP1DTH;
volatile uint8_t  * const IO_USB1_EP2DTL		__attribute__ ((section(".ioview"))) = &USB1_EP2DTL;
volatile uint8_t  * const IO_USB1_EP2DTH		__attribute__ ((section(".ioview"))) = &USB1_EP2DTH;
volatile uint8_t  * const IO_USB1_EP3DTL		__attribute__ ((section(".ioview"))) = &USB1_EP3DTL;
volatile uint8_t  * const IO_USB1_EP3DTH		__attribute__ ((section(".ioview"))) = &USB1_EP3DTH;
volatile uint8_t  * const IO_USB1_EP4DTL		__attribute__ ((section(".ioview"))) = &USB1_EP4DTL;
volatile uint8_t  * const IO_USB1_EP4DTH		__attribute__ ((section(".ioview"))) = &USB1_EP4DTH;
volatile uint8_t  * const IO_USB1_EP5DTL		__attribute__ ((section(".ioview"))) = &USB1_EP5DTL;
volatile uint8_t  * const IO_USB1_EP5DTH		__attribute__ ((section(".ioview"))) = &USB1_EP5DTH;

/* IO_DMAC */
volatile uint32_t * const IO_DMACR				__attribute__ ((section(".ioview"))) = &DMACR;
volatile uint32_t * const IO_DMACA0 			__attribute__ ((section(".ioview"))) = &DMACA0;
volatile uint32_t * const IO_DMACB0 			__attribute__ ((section(".ioview"))) = &DMACB0;
volatile uint32_t * const IO_DMACSA0			__attribute__ ((section(".ioview"))) = &DMACSA0;
volatile uint32_t * const IO_DMACDA0			__attribute__ ((section(".ioview"))) = &DMACDA0;
volatile uint32_t * const IO_DMACA1 			__attribute__ ((section(".ioview"))) = &DMACA1;
volatile uint32_t * const IO_DMACB1 			__attribute__ ((section(".ioview"))) = &DMACB1;
volatile uint32_t * const IO_DMACSA1			__attribute__ ((section(".ioview"))) = &DMACSA1;
volatile uint32_t * const IO_DMACDA1			__attribute__ ((section(".ioview"))) = &DMACDA1;
volatile uint32_t * const IO_DMACA2 			__attribute__ ((section(".ioview"))) = &DMACA2;
volatile stc_dmac_dmaca2_field_t * const IO_DMACA2_B __attribute__ ((section(".ioview"))) = (stc_dmac_dmaca2_field_t *)&DMACA2;
volatile uint32_t * const IO_DMACB2 			__attribute__ ((section(".ioview"))) = &DMACB2;
volatile stc_dmac_dmacb2_field_t * const IO_DMACB2_B __attribute__ ((section(".ioview"))) = (stc_dmac_dmacb2_field_t *)&DMACB2;
volatile uint32_t * const IO_DMACSA2			__attribute__ ((section(".ioview"))) = &DMACSA2;
volatile uint32_t * const IO_DMACDA2			__attribute__ ((section(".ioview"))) = &DMACDA2;
volatile uint32_t * const IO_DMACA3 			__attribute__ ((section(".ioview"))) = &DMACA3;
volatile uint32_t * const IO_DMACB3 			__attribute__ ((section(".ioview"))) = &DMACB3;
volatile uint32_t * const IO_DMACSA3			__attribute__ ((section(".ioview"))) = &DMACSA3;
volatile uint32_t * const IO_DMACDA3			__attribute__ ((section(".ioview"))) = &DMACDA3;
volatile uint32_t * const IO_DMACA4 			__attribute__ ((section(".ioview"))) = &DMACA4;
volatile uint32_t * const IO_DMACB4 			__attribute__ ((section(".ioview"))) = &DMACB4;
volatile uint32_t * const IO_DMACSA4			__attribute__ ((section(".ioview"))) = &DMACSA4;
volatile uint32_t * const IO_DMACDA4			__attribute__ ((section(".ioview"))) = &DMACDA4;
volatile uint32_t * const IO_DMACA5 			__attribute__ ((section(".ioview"))) = &DMACA5;
volatile uint32_t * const IO_DMACB5 			__attribute__ ((section(".ioview"))) = &DMACB5;
volatile uint32_t * const IO_DMACSA5			__attribute__ ((section(".ioview"))) = &DMACSA5;
volatile uint32_t * const IO_DMACDA5			__attribute__ ((section(".ioview"))) = &DMACDA5;
volatile uint32_t * const IO_DMACA6 			__attribute__ ((section(".ioview"))) = &DMACA6;
volatile uint32_t * const IO_DMACB6 			__attribute__ ((section(".ioview"))) = &DMACB6;
volatile uint32_t * const IO_DMACSA6			__attribute__ ((section(".ioview"))) = &DMACSA6;
volatile uint32_t * const IO_DMACDA6			__attribute__ ((section(".ioview"))) = &DMACDA6;
volatile uint32_t * const IO_DMACA7 			__attribute__ ((section(".ioview"))) = &DMACA7;
volatile uint32_t * const IO_DMACB7 			__attribute__ ((section(".ioview"))) = &DMACB7;
volatile uint32_t * const IO_DMACSA7			__attribute__ ((section(".ioview"))) = &DMACSA7;
volatile uint32_t * const IO_DMACDA7			__attribute__ ((section(".ioview"))) = &DMACDA7;

/* IO_CAN ch0-1 */
volatile uint16_t * const IO_CAN0_CTRLR 		__attribute__ ((section(".ioview"))) = &CAN0_CTRLR;
volatile uint16_t * const IO_CAN0_STATR 		__attribute__ ((section(".ioview"))) = &CAN0_STATR;
volatile uint16_t * const IO_CAN0_ERRCNT		__attribute__ ((section(".ioview"))) = &CAN0_ERRCNT;
volatile uint16_t * const IO_CAN0_BTR 			__attribute__ ((section(".ioview"))) = &CAN0_BTR;
volatile uint16_t * const IO_CAN0_INTR			__attribute__ ((section(".ioview"))) = &CAN0_INTR;
volatile uint16_t * const IO_CAN0_TESTR 		__attribute__ ((section(".ioview"))) = &CAN0_TESTR;
volatile uint16_t * const IO_CAN0_BRPER 		__attribute__ ((section(".ioview"))) = &CAN0_BRPER;
volatile uint16_t * const IO_CAN0_IF1CREQ		__attribute__ ((section(".ioview"))) = &CAN0_IF1CREQ;
volatile uint16_t * const IO_CAN0_IF1CMSK		__attribute__ ((section(".ioview"))) = &CAN0_IF1CMSK;
volatile uint16_t * const IO_CAN0_IF1MSK1		__attribute__ ((section(".ioview"))) = &CAN0_IF1MSK1;
volatile uint16_t * const IO_CAN0_IF1MSK2		__attribute__ ((section(".ioview"))) = &CAN0_IF1MSK2;
volatile uint16_t * const IO_CAN0_IF1ARB1		__attribute__ ((section(".ioview"))) = &CAN0_IF1ARB1;
volatile uint16_t * const IO_CAN0_IF1ARB2		__attribute__ ((section(".ioview"))) = &CAN0_IF1ARB2;
volatile uint16_t * const IO_CAN0_IF1MCTR		__attribute__ ((section(".ioview"))) = &CAN0_IF1MCTR;
volatile uint16_t * const IO_CAN0_IF1DTA1LE		__attribute__ ((section(".ioview"))) = &CAN0_IF1DTA1LE;
volatile uint16_t * const IO_CAN0_IF1DTA2LE		__attribute__ ((section(".ioview"))) = &CAN0_IF1DTA2LE;
volatile uint16_t * const IO_CAN0_IF1DTB1LE		__attribute__ ((section(".ioview"))) = &CAN0_IF1DTB1LE;
volatile uint16_t * const IO_CAN0_IF1DTB2LE		__attribute__ ((section(".ioview"))) = &CAN0_IF1DTB2LE;
volatile uint16_t * const IO_CAN0_IF1DTA2BE		__attribute__ ((section(".ioview"))) = &CAN0_IF1DTA2BE;
volatile uint16_t * const IO_CAN0_IF1DTA1BE		__attribute__ ((section(".ioview"))) = &CAN0_IF1DTA1BE;
volatile uint16_t * const IO_CAN0_IF1DTB2BE		__attribute__ ((section(".ioview"))) = &CAN0_IF1DTB2BE;
volatile uint16_t * const IO_CAN0_IF1DTB1BE		__attribute__ ((section(".ioview"))) = &CAN0_IF1DTB1BE;
volatile uint16_t * const IO_CAN0_IF2CREQ		__attribute__ ((section(".ioview"))) = &CAN0_IF2CREQ;
volatile uint16_t * const IO_CAN0_IF2CMSK		__attribute__ ((section(".ioview"))) = &CAN0_IF2CMSK;
volatile uint16_t * const IO_CAN0_IF2MSK1		__attribute__ ((section(".ioview"))) = &CAN0_IF2MSK1;
volatile uint16_t * const IO_CAN0_IF2MSK2		__attribute__ ((section(".ioview"))) = &CAN0_IF2MSK2;
volatile uint16_t * const IO_CAN0_IF2ARB1		__attribute__ ((section(".ioview"))) = &CAN0_IF2ARB1;
volatile uint16_t * const IO_CAN0_IF2ARB2		__attribute__ ((section(".ioview"))) = &CAN0_IF2ARB2;
volatile uint16_t * const IO_CAN0_IF2MCTR		__attribute__ ((section(".ioview"))) = &CAN0_IF2MCTR;
volatile uint16_t * const IO_CAN0_IF2DTA1LE		__attribute__ ((section(".ioview"))) = &CAN0_IF2DTA1LE;
volatile uint16_t * const IO_CAN0_IF2DTA2LE		__attribute__ ((section(".ioview"))) = &CAN0_IF2DTA2LE;
volatile uint16_t * const IO_CAN0_IF2DTB1LE		__attribute__ ((section(".ioview"))) = &CAN0_IF2DTB1LE;
volatile uint16_t * const IO_CAN0_IF2DTB2LE		__attribute__ ((section(".ioview"))) = &CAN0_IF2DTB2LE;
volatile uint16_t * const IO_CAN0_IF2DTA2BE		__attribute__ ((section(".ioview"))) = &CAN0_IF2DTA2BE;
volatile uint16_t * const IO_CAN0_IF2DTA1BE		__attribute__ ((section(".ioview"))) = &CAN0_IF2DTA1BE;
volatile uint16_t * const IO_CAN0_IF2DTB2BE		__attribute__ ((section(".ioview"))) = &CAN0_IF2DTB2BE;
volatile uint16_t * const IO_CAN0_IF2DTB1BE		__attribute__ ((section(".ioview"))) = &CAN0_IF2DTB1BE;

volatile uint16_t * const IO_CAN1_CTRLR 		__attribute__ ((section(".ioview"))) = &CAN1_CTRLR;
volatile uint16_t * const IO_CAN1_STATR 		__attribute__ ((section(".ioview"))) = &CAN1_STATR;
volatile uint16_t * const IO_CAN1_ERRCNT		__attribute__ ((section(".ioview"))) = &CAN1_ERRCNT;
volatile uint16_t * const IO_CAN1_BTR 			__attribute__ ((section(".ioview"))) = &CAN1_BTR;
volatile uint16_t * const IO_CAN1_INTR			__attribute__ ((section(".ioview"))) = &CAN1_INTR;
volatile uint16_t * const IO_CAN1_TESTR 		__attribute__ ((section(".ioview"))) = &CAN1_TESTR;
volatile uint16_t * const IO_CAN1_BRPER 		__attribute__ ((section(".ioview"))) = &CAN1_BRPER;
volatile uint16_t * const IO_CAN1_IF1CREQ		__attribute__ ((section(".ioview"))) = &CAN1_IF1CREQ;
volatile uint16_t * const IO_CAN1_IF1CMSK		__attribute__ ((section(".ioview"))) = &CAN1_IF1CMSK;
volatile uint16_t * const IO_CAN1_IF1MSK1		__attribute__ ((section(".ioview"))) = &CAN1_IF1MSK1;
volatile uint16_t * const IO_CAN1_IF1MSK2		__attribute__ ((section(".ioview"))) = &CAN1_IF1MSK2;
volatile uint16_t * const IO_CAN1_IF1ARB1		__attribute__ ((section(".ioview"))) = &CAN1_IF1ARB1;
volatile uint16_t * const IO_CAN1_IF1ARB2		__attribute__ ((section(".ioview"))) = &CAN1_IF1ARB2;
volatile uint16_t * const IO_CAN1_IF1MCTR		__attribute__ ((section(".ioview"))) = &CAN1_IF1MCTR;
volatile uint16_t * const IO_CAN1_IF1DTA1LE		__attribute__ ((section(".ioview"))) = &CAN1_IF1DTA1LE;
volatile uint16_t * const IO_CAN1_IF1DTA2LE		__attribute__ ((section(".ioview"))) = &CAN1_IF1DTA2LE;
volatile uint16_t * const IO_CAN1_IF1DTB1LE		__attribute__ ((section(".ioview"))) = &CAN1_IF1DTB1LE;
volatile uint16_t * const IO_CAN1_IF1DTB2LE		__attribute__ ((section(".ioview"))) = &CAN1_IF1DTB2LE;
volatile uint16_t * const IO_CAN1_IF1DTA2BE		__attribute__ ((section(".ioview"))) = &CAN1_IF1DTA2BE;
volatile uint16_t * const IO_CAN1_IF1DTA1BE		__attribute__ ((section(".ioview"))) = &CAN1_IF1DTA1BE;
volatile uint16_t * const IO_CAN1_IF1DTB2BE		__attribute__ ((section(".ioview"))) = &CAN1_IF1DTB2BE;
volatile uint16_t * const IO_CAN1_IF1DTB1BE		__attribute__ ((section(".ioview"))) = &CAN1_IF1DTB1BE;
volatile uint16_t * const IO_CAN1_IF2CREQ		__attribute__ ((section(".ioview"))) = &CAN1_IF2CREQ;
volatile uint16_t * const IO_CAN1_IF2CMSK		__attribute__ ((section(".ioview"))) = &CAN1_IF2CMSK;
volatile uint16_t * const IO_CAN1_IF2MSK1		__attribute__ ((section(".ioview"))) = &CAN1_IF2MSK1;
volatile uint16_t * const IO_CAN1_IF2MSK2		__attribute__ ((section(".ioview"))) = &CAN1_IF2MSK2;
volatile uint16_t * const IO_CAN1_IF2ARB1		__attribute__ ((section(".ioview"))) = &CAN1_IF2ARB1;
volatile uint16_t * const IO_CAN1_IF2ARB2		__attribute__ ((section(".ioview"))) = &CAN1_IF2ARB2;
volatile uint16_t * const IO_CAN1_IF2MCTR		__attribute__ ((section(".ioview"))) = &CAN1_IF2MCTR;
volatile uint16_t * const IO_CAN1_IF2DTA1LE		__attribute__ ((section(".ioview"))) = &CAN1_IF2DTA1LE;
volatile uint16_t * const IO_CAN1_IF2DTA2LE		__attribute__ ((section(".ioview"))) = &CAN1_IF2DTA2LE;
volatile uint16_t * const IO_CAN1_IF2DTB1LE		__attribute__ ((section(".ioview"))) = &CAN1_IF2DTB1LE;
volatile uint16_t * const IO_CAN1_IF2DTB2LE		__attribute__ ((section(".ioview"))) = &CAN1_IF2DTB2LE;
volatile uint16_t * const IO_CAN1_IF2DTA2BE		__attribute__ ((section(".ioview"))) = &CAN1_IF2DTA2BE;
volatile uint16_t * const IO_CAN1_IF2DTA1BE		__attribute__ ((section(".ioview"))) = &CAN1_IF2DTA1BE;
volatile uint16_t * const IO_CAN1_IF2DTB2BE		__attribute__ ((section(".ioview"))) = &CAN1_IF2DTB2BE;
volatile uint16_t * const IO_CAN1_IF2DTB1BE		__attribute__ ((section(".ioview"))) = &CAN1_IF2DTB1BE;

/* Variables -----------------------------------------------------------------*/

/* Prototypes ----------------------------------------------------------------*/

/* Functions -----------------------------------------------------------------*/

/* End Of File ---------------------------------------------------------------*/
