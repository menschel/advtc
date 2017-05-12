;*********************************************************************
; Fuzzy Logic Traction Controller named "ADVanced Traction Control" 
; As HCS12 doesnt support fuzzy mnemonics any more, only normal 
; mnemonics are used. Originally written for Megasquirt 2
; This header must appear on all derivates of this code
; Distributed under Gnu Public License, e.g. not for sale, free usage
; for non-profit and educational purposes
; http://www.gnu.org/licenses/gpl.html
;*********************************************************************
;File: advtc.s
;Author: (c) 2009 - Patrick Menschel
;Size: 339 bytes - 200cylces estimated, some more for fuel cut
;*********************************************************************
;Programm flow check, no_tc verified! no time write advance verified!
;time,calc,writeadvance verified! SP change is 0 on every programm path
;*********************************************************************
.sect .text3a

.globl outpc,flash9,advTcRetardValue,advTcLastSlipValue,advTcLastundrivenAdc, advtc_fuzzy
.globl advTcMaxActiveTimer ;timer integer

.equ flash9.advTcOn,       flash9+0x3E4
.equ flash9.advTcFuzzySingletons, flash9+0x3E5
.equ flash9.advTcMaxRetard, flash9+0x3E9
.equ flash9.advTcMaxActiveTime, flash9+0x3EA
.equ drivenwheeladc,       outpc+0x80 ;driven wheel adc6 for the moment
.equ undrivenwheeladc,     outpc+0x82 ;undriven wheel adc7 for the moment
.equ outpc.status3,        outpc+0x50
.equ outpc.status4,        outpc+0x51

; note lt entry points for limit output

;stuff to make it a simple function call
.globl lmms,advTcUpdateClk

;function call assumes that a 16bit pointer to retard variable (long/32bit) is in D, this is usually 16-19,SP

;update to use deltadeltamu instead
;.globl advTcLastDeltaUndrivenAdc


advtc_fuzzy:
   pshd ;push pointer to lsum on stack										+2

;on/off switch
   brclr flash9.advTcOn,0x01,advtc_no_tc_end;branch to end, cleanup stack, do nothing

   ldd advTcMaxActiveTimer
   cpd flash9.advTcMaxActiveTime
   blo advtc_check_slip
   movb #255, outpc.status4 ;show sensor faulty in status4
   bra advtc_no_tc_end ;sensor faulty, deactivate until power cycle
   
advtc_check_slip:
;check for slip, push rpm diff for later calc of current slip
   ldd drivenwheeladc
   subd undrivenwheeladc
   bhi advtc_setup_memory ;we have slip so continue

;************************************************************ 
;no regulation stage - shortcut for no Slip
advtc_no_tc:
;branch for no slip reset runtime variables here, keep record of undriven wheel
   clr advTcRetardValue
   movw #0, advTcLastSlipValue
   movw undrivenwheeladc,advTcLastundrivenAdc
;   movw #0, advTcLastDeltaUndrivenAdc
   movw #0, advTcMaxActiveTimer ;reset active timer

advtc_no_tc_end:
   leas 2,SP ; restore pointer from lsum push								-2, SP change is 2-2=0
   rtc


advtc_setup_memory:
   leas -8,SP ;set up stack memory used for fuzzy calculations, this was fuzzyinputoutput array previously +8
   pshd ; push adc difference aka slip on stack										+2

;************************************************************ 
;time control stage here   

   sei ;disable interrupts during time get
   ldd lmms     ;high word
   ldx lmms+0x2 ;low word
   cli ;enable interrupts again

;compare times
   cpd advTcUpdateClk ;compare high word
   exg d,x ;exchange d and x, does not change CCR
   bgt advtc_write_next_time
   
   cpd advTcUpdateClk+0x2 ;compare low word
   bgt advtc_write_next_time
   
   leas 10,SP ;correct SP for adc difference and calc buffer, no controller step this time   -10   
   bra advtc_write_advance ;branch to end, write last value on advance
   ;timer not toggled yet, write last value
   
advtc_write_next_time:
;timer is toggled, so save time of this controller step   
   addd #78 ;78ticks = 10ms
   std advTcUpdateClk+0x2 ;write low word
   bcc advtc_write_high_time ;if no carry do not increase high word   
   inx ;increase high word
   
advtc_write_high_time:
   stx advTcUpdateClk
   ;note without saving the high byte the 10ms timer will fail when high byte is higher but no carry is produced here 

   clra
   brclr flash9.advTcOn,0x80,advtc_calc_inputs ;skip timer if bit cleared


advtc_increase_active_timer:
   ldd advTcMaxActiveTimer
   addd #1                 ;increase active 10ms timer
   std advTcMaxActiveTimer

;************************************************************ 
;input limit and fuzzification stage here
   
advtc_calc_inputs:
;calculate change of undrivenwheel aka deltamu
   ldd undrivenwheeladc
   pshd ;buffer undriven wheel												+2
   subd advTcLastundrivenAdc
   
;use another derivate on mu
;   pshd ;buffer to stack for a moment
;   subd advTcLastDeltaUndrivenAdc
;   std outpc+0x54;status5
;   movw 0,SP, advTcLastDeltaUndrivenAdc
;   leas 2,SP
   
;limit to 8bit signed
   cpd #0xff80 ; -128
   bge advtc_lt
   ldd #0xff80 ; -128
   bra advtc_lt2

advtc_lt:
   cpd #0x7f; 127
   ble advtc_lt2
   ldd #0x7f; 127

advtc_lt2:
;d is now 8bit signed range -128 to 127
;save undriven adc for next loop
   movw 0,SP , advTcLastundrivenAdc
;calculate the corresponding fuzzyinputs deltamupos/neg

   clra ;discard high byte
;   leax 4,SP ; set pointer to 8byte calculation buffer on stack
   pshb ; push low byte on stack
   ldab #0x7f ; 127
   subb 0,SP ; b on stack
;   stab 0,X ; deltamunegative! 127 - (-128 to 127)= 0 to 255 8bit unsigned range
   stab 5,SP ; deltamunegative! 127 - (-128 to 127)= 0 to 255 8bit unsigned range

   pulb
   addb #0x80 ; 128
;   stab 1,X ; deltamupositive! 128 + (-128 to 127)= 0 to 255 8bit unsigned range
   stab 5,SP ; deltamupositive! 128 + (-128 to 127)= 0 to 255 8bit unsigned range

;calculate change of slip in percentage*10  aka deltalambda   
   pulx ;pull undriven wheel buffer											-2
   inx ;never divide by 0

   puld; already have adc difference aka slip on stack pullfrom stack,decrease SP by 2		-2
   subd #1 ; prevent reach of slip state when undriven wheel is at 0 Speed aka no reaction on adc flickering  
   ldy #1000
   emul

   ediv ;Y = (1000*(adcdiff-1))/undrivenadc = slip percentage * 10,D=Remainder
   tfr Y,D
;limit output to 16bit signed, we know its positive
   cpd #0x7fff ; 32767
   ble advtc_lt3
   ldd #0x7fff ; 32767

advtc_lt3:
   pshd ;buffer slip percentage*10, the stack pointer is kept until output calculation			+2
   subd advTcLastSlipValue
;limit output to 8bit signed
   cpd #0xff80 ; -128
   bge advtc_lt4
   ldd #0xff80 ; -128
   bra advtc_lt5
advtc_lt4:
   cpd #0x7f; 127
   ble advtc_lt5
   ldd #0x7f; 127
advtc_lt5:
;now store advTcLastSlipValue for next loop
   movw 0,SP , advTcLastSlipValue
    
;calculate the corresponding fuzzyinputs deltalambdapos/neg

   pshb   
   ldab #0x7f ; 127
   subb 0,SP

   stab 5,SP ;!deltalambdanegative  127 - (-128 to 127)= 0 to 255 8bit unsigned range   
   pulb
   addb #0x80 ; 128

   stab 5,SP ;!deltalambdapositive  128 + (-128 to 127)= 0 to 255 8bit unsigned range

;************************************************************ 
FUZZYEVAL:
; rule evaluation here

   clrb
   ldaa 4,SP
   mina 2,SP
   staa 6,SP				; rule1 if deltamunegative and deltalambdanegative >> advance much higher   

   ldaa 5,SP
   mina 2,SP
   staa 9,SP				; rule3 if deltamunegative and deltalambdapositive >> retard much lower

   ldaa 4,SP
   mina 3,SP
   staa 7,SP				; rule2 if deltamupositive and deltalambdanegative >> advance some higher

   ldaa 5,SP
   mina 3,SP
   staa 8,SP				; rule4 if deltamupositive and deltalambdapositive >> retard some lower


;now do defuzzification aka true output step calculation
;first init variables
   ldd #0
   std 0,SP ;we still have 2bytes buffer on stack and did not reset the stack pointer on purpose to save one instruction this way
   pshd;																+2
   pshd;																+2

   ;describtion of stack inside advtc_defuz
   ;7,SP 8byte calc buffer, now only 4byte fuzzy outputs
   ;5,SP buffer pointer to Singletons 
   ;3,SP is multiplied truth
   ;1,SP is overall truth    
   ;0,SP is loop variable   
   
   ldab #4 ; ctmp1 loop variable
;now set pointers
   leax 10,SP ;set pointer to outputs 6,SP(start of buffer) and 4(offset in buffer)=10,SP   
   ldy #flash9.advTcFuzzySingletons


;************************************************************  

advtc_defuz:
;loop, store loop variable
   pshb  

;now get singleton   
   ldab 0,Y
;signed extent to 16bit   
   sex B,D
   
   sty 5,SP ;buffer pointer to Singletons

   tfr D,Y ;move D to Y
   ;now have a 8bit singleton transformed to 16bit in Y
   
; now get output truth value in D

   clra ;clear A, we know truth is positive
   
   ldab 0,X ;load output truth
   
   addd 1,SP ;add to overall truth
      
   std 1,SP ;store overall truth

;get output truth value again     
   clra
   ldab 0,X
      
   emuls ;signed multiply d*y

   addd 3,SP ;add to multiplied truth

   std  3,SP ;store multiplied truth

   ldy 5,SP ;restore pointer to Singletions
   
   inx ; increase both pointers
   iny
   
   pulb ;restore loop variable for dbne instruction

   dbne B,advtc_defuz
;************************************************************   
;load loop variable, autodec and branch back to start if not 0

   pulx ;pull overall truth																		-2
   puld ;pull multiplied truth																	-2
   leas 10,SP ;stack cleanup for pointer and fuzzy calculations this was fuzzyinputoutput array previously -10   
   ;stack is now back in the state, it had before the function call!
   
   idivs ;signed integer division d/x - now have real output in X, Remainder in D
   
;compute real output
   tfr X,D
;add this step to overall output
   clra
   addb advTcRetardValue
   
;************************************************************
;output limitagion stage
    
;output limitation 0
   cmpb #0
   bge advtc_lt6
   movb #0, advTcRetardValue
   bra advtc_write_advance
   
advtc_lt6:
;output limitation advTcMaxRetard
   cmpb flash9.advTcMaxRetard
   blt advtc_lt7 ;was ble previously
   movb flash9.advTcMaxRetard, advTcRetardValue
   brclr flash9.advTcOn,0x06,advtc_write_advance
   bset outpc.status3,#0x04 ; status3_traction_active, toggle fuel cut
   bra advtc_write_advance
   
advtc_lt7:
;output did not touch the borders
   stab advTcRetardValue
   brclr flash9.advTcOn,0x06,advtc_write_advance
   bclr outpc.status3,#0x04 ; status3_traction_active, restore fuel 

;************************************************************ 
;output write stage - shortcut for timer not toggled    
advtc_write_advance:
;temporary do lsum sub here
   puly ;pointer to lsum there												-2, SP change is2+10-10-2=0
   ldx 0,Y ;high word to X
   ldd 2,Y ;low word to D
   subb advTcRetardValue ;sub it
   sbca #0 ;sub carry
   std 2,Y ;store it back 
   bcc advtc_write_advance_end ;if no borrow branch to end
   dex
   stx 0,Y

advtc_write_advance_end: ;dummy entry point
   movb advTcRetardValue,outpc.status4 ;log output to status4
   rtc
