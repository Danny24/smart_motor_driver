
_interrupt:

;SmartMotorDriver.c,13 :: 		void interrupt() iv 0x0004 ics ICS_AUTO //interrupts
;SmartMotorDriver.c,15 :: 		if (INTCON.f0 == 1 && IOCAF.f4 == 1) //interrupt on change on RA4 trigered
	BTFSS      INTCON+0, 0
	GOTO       L_interrupt2
	BTFSS      IOCAF+0, 4
	GOTO       L_interrupt2
L__interrupt17:
;SmartMotorDriver.c,17 :: 		INTCON.f3 = 0; //disable on change interrupts
	BCF        INTCON+0, 3
;SmartMotorDriver.c,18 :: 		counter ++;  //increment counter one in one
	INCF       _counter+0, 1
;SmartMotorDriver.c,19 :: 		IOCAF.f4 = 0; //clear interrupt flags
	BCF        IOCAF+0, 4
;SmartMotorDriver.c,20 :: 		INTCON.f3 = 1; //enable on change interrupts
	BSF        INTCON+0, 3
;SmartMotorDriver.c,21 :: 		}
L_interrupt2:
;SmartMotorDriver.c,22 :: 		if(PIR1.f0 == 1) //timer1 interrupt, called every 65.536ms
	BTFSS      PIR1+0, 0
	GOTO       L_interrupt3
;SmartMotorDriver.c,24 :: 		INTCON.f3 = 0; //disable on change interrupts
	BCF        INTCON+0, 3
;SmartMotorDriver.c,25 :: 		T1CON.f0 =  0; //stop timer1
	BCF        T1CON+0, 0
;SmartMotorDriver.c,26 :: 		rpm = (counter * 300)/gear;  //calculate rpm  (multiplied 15 interrupts in 1 second, divided 3 encoder interrupts per lap, multiplied by 60 to convert to minutes, divided by gear ratio)
	MOVF       _counter+0, 0
	MOVWF      R0
	CLRF       R1
	MOVLW      44
	MOVWF      R4
	MOVLW      1
	MOVWF      R5
	CALL       _Mul_16X16_U+0
	MOVF       _gear+0, 0
	MOVWF      R4
	MOVF       _gear+1, 0
	MOVWF      R5
	CALL       _Div_16X16_U+0
	MOVF       R0, 0
	MOVWF      _rpm+0
	MOVF       R1, 0
	MOVWF      _rpm+1
;SmartMotorDriver.c,27 :: 		counter = 0;  //clear counter
	CLRF       _counter+0
;SmartMotorDriver.c,28 :: 		INTCON.f3 = 1; //enable on change interrupts
	BSF        INTCON+0, 3
;SmartMotorDriver.c,29 :: 		PIR1.f0 = 0; //clear interrutp flag
	BCF        PIR1+0, 0
;SmartMotorDriver.c,30 :: 		T1CON.f0 =  1; //start timer1
	BSF        T1CON+0, 0
;SmartMotorDriver.c,31 :: 		}
L_interrupt3:
;SmartMotorDriver.c,32 :: 		}
L_end_interrupt:
L__interrupt19:
	RETFIE     %s
; end of _interrupt

_main:

;SmartMotorDriver.c,37 :: 		void main()
;SmartMotorDriver.c,39 :: 		OSCCON = 0b11110000; //configure internal oscilator fro 32Mhz
	MOVLW      240
	MOVWF      OSCCON+0
;SmartMotorDriver.c,40 :: 		TRISA = 0b00011100;  //configure IO
	MOVLW      28
	MOVWF      TRISA+0
;SmartMotorDriver.c,41 :: 		ANSELA = 0b00000000; //analog functions of pins disabled
	CLRF       ANSELA+0
;SmartMotorDriver.c,42 :: 		WPUA = 0b00011110;   //configure weak pull-ups on input pins
	MOVLW      30
	MOVWF      WPUA+0
;SmartMotorDriver.c,43 :: 		OPTION_REG.f7 = 0;   //enable weak pull-ups
	BCF        OPTION_REG+0, 7
;SmartMotorDriver.c,44 :: 		APFCON.f0 = 1;       //select RA5 as CCP output pin
	BSF        APFCON+0, 0
;SmartMotorDriver.c,45 :: 		LATA.f0 = 0;         //put motor direction pin to low
	BCF        LATA+0, 0
;SmartMotorDriver.c,46 :: 		PWM1_init(50000);    //confifure pwm frecuency
	BCF        T2CON+0, 0
	BCF        T2CON+0, 1
	MOVLW      159
	MOVWF      PR2+0
	CALL       _PWM1_Init+0
;SmartMotorDriver.c,47 :: 		PWM1_start();        //start pwm module
	CALL       _PWM1_Start+0
;SmartMotorDriver.c,48 :: 		PWM1_set_duty(0);    //put duty of pwm to 0
	CLRF       FARG_PWM1_Set_Duty_new_duty+0
	CALL       _PWM1_Set_Duty+0
;SmartMotorDriver.c,49 :: 		IOCAN.f4 = 1;        //configure interrupt on falling edge for rpm meter
	BSF        IOCAN+0, 4
;SmartMotorDriver.c,50 :: 		INTCON = 0b01001000; //enables interrupts
	MOVLW      72
	MOVWF      INTCON+0
;SmartMotorDriver.c,51 :: 		T1CON = 0b00110100;  //configure timer1 to run at 1 MHz
	MOVLW      52
	MOVWF      T1CON+0
;SmartMotorDriver.c,52 :: 		PIE1.f0 =  1;        //enable timer1 interrupt
	BSF        PIE1+0, 0
;SmartMotorDriver.c,53 :: 		T1CON.f0 =  1;       //start timer1
	BSF        T1CON+0, 0
;SmartMotorDriver.c,54 :: 		INTCON.f7 = 1;       //run interrupts
	BSF        INTCON+0, 7
;SmartMotorDriver.c,56 :: 		M_control(0);
	CLRF       FARG_M_control_ctr+0
	CLRF       FARG_M_control_ctr+1
	CALL       _M_control+0
;SmartMotorDriver.c,57 :: 		Soft_UART_Init(&PORTA, 2, 1, 9600, 0);
	MOVLW      PORTA+0
	MOVWF      FARG_Soft_UART_Init_port+0
	MOVLW      hi_addr(PORTA+0)
	MOVWF      FARG_Soft_UART_Init_port+1
	MOVLW      2
	MOVWF      FARG_Soft_UART_Init_rx_pin+0
	MOVLW      1
	MOVWF      FARG_Soft_UART_Init_tx_pin+0
	MOVLW      128
	MOVWF      FARG_Soft_UART_Init_baud_rate+0
	MOVLW      37
	MOVWF      FARG_Soft_UART_Init_baud_rate+1
	CLRF       FARG_Soft_UART_Init_baud_rate+2
	CLRF       FARG_Soft_UART_Init_baud_rate+3
	CLRF       FARG_Soft_UART_Init_inverted+0
	CALL       _Soft_UART_Init+0
;SmartMotorDriver.c,59 :: 		while(1)
L_main4:
;SmartMotorDriver.c,61 :: 		PID(100);
	MOVLW      100
	MOVWF      FARG_PID_ctr+0
	MOVLW      0
	MOVWF      FARG_PID_ctr+1
	CALL       _PID+0
;SmartMotorDriver.c,62 :: 		IntToStr(rpm, txt);
	MOVF       _rpm+0, 0
	MOVWF      FARG_IntToStr_input+0
	MOVF       _rpm+1, 0
	MOVWF      FARG_IntToStr_input+1
	MOVLW      _txt+0
	MOVWF      FARG_IntToStr_output+0
	MOVLW      hi_addr(_txt+0)
	MOVWF      FARG_IntToStr_output+1
	CALL       _IntToStr+0
;SmartMotorDriver.c,63 :: 		for (i=0;i<strlen(txt);i++)
	CLRF       _i+0
	CLRF       _i+1
L_main6:
	MOVLW      _txt+0
	MOVWF      FARG_strlen_s+0
	MOVLW      hi_addr(_txt+0)
	MOVWF      FARG_strlen_s+1
	CALL       _strlen+0
	MOVLW      128
	XORWF      _i+1, 0
	MOVWF      R2
	MOVLW      128
	XORWF      R1, 0
	SUBWF      R2, 0
	BTFSS      STATUS+0, 2
	GOTO       L__main21
	MOVF       R0, 0
	SUBWF      _i+0, 0
L__main21:
	BTFSC      STATUS+0, 0
	GOTO       L_main7
;SmartMotorDriver.c,65 :: 		Soft_UART_Write(txt[i]);
	MOVLW      _txt+0
	ADDWF      _i+0, 0
	MOVWF      FSR0L
	MOVLW      hi_addr(_txt+0)
	ADDWFC     _i+1, 0
	MOVWF      FSR0H
	MOVF       INDF0+0, 0
	MOVWF      FARG_Soft_UART_Write_udata+0
	CALL       _Soft_UART_Write+0
;SmartMotorDriver.c,63 :: 		for (i=0;i<strlen(txt);i++)
	INCF       _i+0, 1
	BTFSC      STATUS+0, 2
	INCF       _i+1, 1
;SmartMotorDriver.c,66 :: 		}
	GOTO       L_main6
L_main7:
;SmartMotorDriver.c,67 :: 		Soft_UART_Write(10);
	MOVLW      10
	MOVWF      FARG_Soft_UART_Write_udata+0
	CALL       _Soft_UART_Write+0
;SmartMotorDriver.c,68 :: 		Soft_UART_Write(13);
	MOVLW      13
	MOVWF      FARG_Soft_UART_Write_udata+0
	CALL       _Soft_UART_Write+0
;SmartMotorDriver.c,69 :: 		delay_ms(100);
	MOVLW      5
	MOVWF      R11
	MOVLW      15
	MOVWF      R12
	MOVLW      241
	MOVWF      R13
L_main9:
	DECFSZ     R13, 1
	GOTO       L_main9
	DECFSZ     R12, 1
	GOTO       L_main9
	DECFSZ     R11, 1
	GOTO       L_main9
;SmartMotorDriver.c,70 :: 		}
	GOTO       L_main4
;SmartMotorDriver.c,71 :: 		}
L_end_main:
	GOTO       $+0
; end of _main

_PID:

;SmartMotorDriver.c,73 :: 		void PID(int ctr) //PID calculation function
;SmartMotorDriver.c,75 :: 		int error = ctr-rpm; //calculate actual error
	MOVF       _rpm+0, 0
	SUBWF      FARG_PID_ctr+0, 0
	MOVWF      FLOC__PID+2
	MOVF       _rpm+1, 0
	SUBWFB     FARG_PID_ctr+1, 0
	MOVWF      FLOC__PID+3
;SmartMotorDriver.c,76 :: 		int PID = error*kp;     // calculate proportional gain
	MOVF       FLOC__PID+2, 0
	MOVWF      R0
	MOVF       FLOC__PID+3, 0
	MOVWF      R1
	MOVF       _kp+0, 0
	MOVWF      R4
	MOVLW      0
	BTFSC      R4, 7
	MOVLW      255
	MOVWF      R5
	CALL       _Mul_16X16_U+0
	MOVF       R0, 0
	MOVWF      FLOC__PID+0
	MOVF       R1, 0
	MOVWF      FLOC__PID+1
	MOVF       FLOC__PID+0, 0
	MOVWF      PID_PID_L0+0
	MOVF       FLOC__PID+1, 0
	MOVWF      PID_PID_L0+1
;SmartMotorDriver.c,77 :: 		accumulator += error;  // calculate accumulator, is sum of errors
	MOVF       FLOC__PID+2, 0
	ADDWF      _accumulator+0, 0
	MOVWF      R0
	MOVF       FLOC__PID+3, 0
	ADDWFC     _accumulator+1, 0
	MOVWF      R1
	MOVF       R0, 0
	MOVWF      _accumulator+0
	MOVF       R1, 0
	MOVWF      _accumulator+1
;SmartMotorDriver.c,78 :: 		PID += ki*accumulator; // add integral gain and error accumulator
	MOVF       _ki+0, 0
	MOVWF      R4
	MOVLW      0
	BTFSC      R4, 7
	MOVLW      255
	MOVWF      R5
	CALL       _Mul_16X16_U+0
	MOVF       R0, 0
	ADDWF      FLOC__PID+0, 1
	MOVF       R1, 0
	ADDWFC     FLOC__PID+1, 1
	MOVF       FLOC__PID+0, 0
	MOVWF      PID_PID_L0+0
	MOVF       FLOC__PID+1, 0
	MOVWF      PID_PID_L0+1
;SmartMotorDriver.c,79 :: 		PID += kd*(error-lasterror); //add differential gain
	MOVF       _lasterror+0, 0
	SUBWF      FLOC__PID+2, 0
	MOVWF      R0
	MOVF       _lasterror+1, 0
	SUBWFB     FLOC__PID+3, 0
	MOVWF      R1
	MOVF       _kd+0, 0
	MOVWF      R4
	MOVLW      0
	BTFSC      R4, 7
	MOVLW      255
	MOVWF      R5
	CALL       _Mul_16X16_U+0
	MOVF       R0, 0
	ADDWF      FLOC__PID+0, 0
	MOVWF      R2
	MOVF       R1, 0
	ADDWFC     FLOC__PID+1, 0
	MOVWF      R3
	MOVF       R2, 0
	MOVWF      PID_PID_L0+0
	MOVF       R3, 0
	MOVWF      PID_PID_L0+1
;SmartMotorDriver.c,80 :: 		lasterror = error; //save the error to the next iteration
	MOVF       FLOC__PID+2, 0
	MOVWF      _lasterror+0
	MOVF       FLOC__PID+3, 0
	MOVWF      _lasterror+1
;SmartMotorDriver.c,81 :: 		if(PID>=255)   //next we guarantee that the PID value is in PWM range
	MOVLW      128
	XORWF      R3, 0
	MOVWF      R0
	MOVLW      128
	SUBWF      R0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__PID23
	MOVLW      255
	SUBWF      R2, 0
L__PID23:
	BTFSS      STATUS+0, 0
	GOTO       L_PID10
;SmartMotorDriver.c,83 :: 		PID = 255;
	MOVLW      255
	MOVWF      PID_PID_L0+0
	CLRF       PID_PID_L0+1
;SmartMotorDriver.c,84 :: 		}
L_PID10:
;SmartMotorDriver.c,85 :: 		if(PID<=-255)
	MOVLW      128
	XORLW      255
	MOVWF      R0
	MOVLW      128
	XORWF      PID_PID_L0+1, 0
	SUBWF      R0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__PID24
	MOVF       PID_PID_L0+0, 0
	SUBLW      1
L__PID24:
	BTFSS      STATUS+0, 0
	GOTO       L_PID11
;SmartMotorDriver.c,87 :: 		PID = -255;
	MOVLW      1
	MOVWF      PID_PID_L0+0
	MOVLW      255
	MOVWF      PID_PID_L0+1
;SmartMotorDriver.c,88 :: 		}
L_PID11:
;SmartMotorDriver.c,89 :: 		M_control(PID);
	MOVF       PID_PID_L0+0, 0
	MOVWF      FARG_M_control_ctr+0
	MOVF       PID_PID_L0+1, 0
	MOVWF      FARG_M_control_ctr+1
	CALL       _M_control+0
;SmartMotorDriver.c,90 :: 		}
L_end_PID:
	RETURN
; end of _PID

_M_control:

;SmartMotorDriver.c,92 :: 		void M_control(int ctr) //motor control function
;SmartMotorDriver.c,94 :: 		if(abs(ctr) > 255)  //if the value is bigger than 8 bits...
	MOVF       FARG_M_control_ctr+0, 0
	MOVWF      FARG_abs_a+0
	MOVF       FARG_M_control_ctr+1, 0
	MOVWF      FARG_abs_a+1
	CALL       _abs+0
	MOVLW      128
	MOVWF      R2
	MOVLW      128
	XORWF      R1, 0
	SUBWF      R2, 0
	BTFSS      STATUS+0, 2
	GOTO       L__M_control26
	MOVF       R0, 0
	SUBLW      255
L__M_control26:
	BTFSC      STATUS+0, 0
	GOTO       L_M_control12
;SmartMotorDriver.c,96 :: 		return;   //exit function
	GOTO       L_end_M_control
;SmartMotorDriver.c,97 :: 		}
L_M_control12:
;SmartMotorDriver.c,100 :: 		if (ctr == 0) //stop the motor
	MOVLW      0
	XORWF      FARG_M_control_ctr+1, 0
	BTFSS      STATUS+0, 2
	GOTO       L__M_control27
	MOVLW      0
	XORWF      FARG_M_control_ctr+0, 0
L__M_control27:
	BTFSS      STATUS+0, 2
	GOTO       L_M_control14
;SmartMotorDriver.c,102 :: 		PWM1_set_duty(ctr);
	MOVF       FARG_M_control_ctr+0, 0
	MOVWF      FARG_PWM1_Set_Duty_new_duty+0
	CALL       _PWM1_Set_Duty+0
;SmartMotorDriver.c,103 :: 		}
L_M_control14:
;SmartMotorDriver.c,104 :: 		if (ctr < 0)  //clockwise turn set and set the pwm duty
	MOVLW      128
	XORWF      FARG_M_control_ctr+1, 0
	MOVWF      R0
	MOVLW      128
	SUBWF      R0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__M_control28
	MOVLW      0
	SUBWF      FARG_M_control_ctr+0, 0
L__M_control28:
	BTFSC      STATUS+0, 0
	GOTO       L_M_control15
;SmartMotorDriver.c,106 :: 		LATA.f0 = 0;
	BCF        LATA+0, 0
;SmartMotorDriver.c,107 :: 		PWM1_set_duty(ctr);
	MOVF       FARG_M_control_ctr+0, 0
	MOVWF      FARG_PWM1_Set_Duty_new_duty+0
	CALL       _PWM1_Set_Duty+0
;SmartMotorDriver.c,108 :: 		}
L_M_control15:
;SmartMotorDriver.c,109 :: 		if (ctr > 0)  //counter clockwise turn set and set the pwm duty
	MOVLW      128
	MOVWF      R0
	MOVLW      128
	XORWF      FARG_M_control_ctr+1, 0
	SUBWF      R0, 0
	BTFSS      STATUS+0, 2
	GOTO       L__M_control29
	MOVF       FARG_M_control_ctr+0, 0
	SUBLW      0
L__M_control29:
	BTFSC      STATUS+0, 0
	GOTO       L_M_control16
;SmartMotorDriver.c,111 :: 		LATA.f0 = 1;
	BSF        LATA+0, 0
;SmartMotorDriver.c,112 :: 		PWM1_set_duty(ctr);
	MOVF       FARG_M_control_ctr+0, 0
	MOVWF      FARG_PWM1_Set_Duty_new_duty+0
	CALL       _PWM1_Set_Duty+0
;SmartMotorDriver.c,113 :: 		}
L_M_control16:
;SmartMotorDriver.c,115 :: 		}
L_end_M_control:
	RETURN
; end of _M_control
