
# 1 "main.c"


# 10
#pragma config FOSC = INTOSC
#pragma config WDTE = OFF
#pragma config PWRTE = ON
#pragma config MCLRE = ON
#pragma config CP = OFF
#pragma config CPD = OFF
#pragma config BOREN = ON
#pragma config CLKOUTEN = OFF
#pragma config IESO = OFF
#pragma config FCMEN = OFF


#pragma config WRT = OFF
#pragma config PLLEN = ON
#pragma config STVREN = ON
#pragma config BORV = LO
#pragma config LVP = OFF

# 4 "C:\Program Files (x86)\Microchip\xc8\v2.00\pic\include\__size_t.h"
typedef unsigned size_t;

# 7 "C:\Program Files (x86)\Microchip\xc8\v2.00\pic\include\c90\stdarg.h"
typedef void * va_list[1];

#pragma intrinsic(__va_start)
extern void * __va_start(void);

#pragma intrinsic(__va_arg)
extern void * __va_arg(void *, ...);

# 43 "C:\Program Files (x86)\Microchip\xc8\v2.00\pic\include\c90\stdio.h"
struct __prbuf
{
char * ptr;
void (* func)(char);
};

# 29 "C:\Program Files (x86)\Microchip\xc8\v2.00\pic\include\c90\errno.h"
extern int errno;

# 12 "C:\Program Files (x86)\Microchip\xc8\v2.00\pic\include\c90\conio.h"
extern void init_uart(void);

extern char getch(void);
extern char getche(void);
extern void putch(char);
extern void ungetch(char);

extern __bit kbhit(void);

# 23
extern char * cgets(char *);
extern void cputs(const char *);

# 88 "C:\Program Files (x86)\Microchip\xc8\v2.00\pic\include\c90\stdio.h"
extern int cprintf(char *, ...);
#pragma printf_check(cprintf)



extern int _doprnt(struct __prbuf *, const register char *, register va_list);


# 180
#pragma printf_check(vprintf) const
#pragma printf_check(vsprintf) const

extern char * gets(char *);
extern int puts(const char *);
extern int scanf(const char *, ...) __attribute__((unsupported("scanf() is not supported by this compiler")));
extern int sscanf(const char *, const char *, ...) __attribute__((unsupported("sscanf() is not supported by this compiler")));
extern int vprintf(const char *, va_list) __attribute__((unsupported("vprintf() is not supported by this compiler")));
extern int vsprintf(char *, const char *, va_list) __attribute__((unsupported("vsprintf() is not supported by this compiler")));
extern int vscanf(const char *, va_list ap) __attribute__((unsupported("vscanf() is not supported by this compiler")));
extern int vsscanf(const char *, const char *, va_list) __attribute__((unsupported("vsscanf() is not supported by this compiler")));

#pragma printf_check(printf) const
#pragma printf_check(sprintf) const
extern int sprintf(char *, const char *, ...);
extern int printf(const char *, ...);

# 7 "C:\Program Files (x86)\Microchip\xc8\v2.00\pic\include\c90\stdlib.h"
typedef unsigned short wchar_t;

# 15
typedef struct {
int rem;
int quot;
} div_t;
typedef struct {
unsigned rem;
unsigned quot;
} udiv_t;
typedef struct {
long quot;
long rem;
} ldiv_t;
typedef struct {
unsigned long quot;
unsigned long rem;
} uldiv_t;

# 65
extern double atof(const char *);
extern double strtod(const char *, const char **);
extern int atoi(const char *);
extern unsigned xtoi(const char *);
extern long atol(const char *);

# 73
extern long strtol(const char *, char **, int);

extern int rand(void);
extern void srand(unsigned int);
extern void * calloc(size_t, size_t);
extern div_t div(int numer, int denom);
extern udiv_t udiv(unsigned numer, unsigned denom);
extern ldiv_t ldiv(long numer, long denom);
extern uldiv_t uldiv(unsigned long numer,unsigned long denom);

# 85
extern unsigned long _lrotl(unsigned long value, unsigned int shift);
extern unsigned long _lrotr(unsigned long value, unsigned int shift);
extern unsigned int _rotl(unsigned int value, unsigned int shift);
extern unsigned int _rotr(unsigned int value, unsigned int shift);




extern void * malloc(size_t);
extern void free(void *);
extern void * realloc(void *, size_t);


# 13 "C:\Program Files (x86)\Microchip\xc8\v2.00\pic\include\c90\xc8debug.h"
#pragma intrinsic(__builtin_software_breakpoint)
extern void __builtin_software_breakpoint(void);

# 104 "C:\Program Files (x86)\Microchip\xc8\v2.00\pic\include\c90\stdlib.h"
extern int atexit(void (*)(void));
extern char * getenv(const char *);
extern char ** environ;
extern int system(char *);
extern void qsort(void *, size_t, size_t, int (*)(const void *, const void *));
extern void * bsearch(const void *, void *, size_t, size_t, int(*)(const void *, const void *));
extern int abs(int);
extern long labs(long);

extern char * itoa(char * buf, int val, int base);
extern char * utoa(char * buf, unsigned val, int base);




extern char * ltoa(char * buf, long val, int base);
extern char * ultoa(char * buf, unsigned long val, int base);

extern char * ftoa(float f, int * status);

# 18 "C:\Program Files (x86)\Microchip\xc8\v2.00\pic\include\xc.h"
extern const char __xc8_OPTIM_SPEED;

extern double __fpnormalize(double);

# 52 "C:\Program Files (x86)\Microchip\xc8\v2.00\pic\include\pic12f1840.h"
extern volatile unsigned char INDF0 __at(0x000);

asm("INDF0 equ 00h");


typedef union {
struct {
unsigned INDF0 :8;
};
} INDF0bits_t;
extern volatile INDF0bits_t INDF0bits __at(0x000);

# 72
extern volatile unsigned char INDF1 __at(0x001);

asm("INDF1 equ 01h");


typedef union {
struct {
unsigned INDF1 :8;
};
} INDF1bits_t;
extern volatile INDF1bits_t INDF1bits __at(0x001);

# 92
extern volatile unsigned char PCL __at(0x002);

asm("PCL equ 02h");


typedef union {
struct {
unsigned PCL :8;
};
} PCLbits_t;
extern volatile PCLbits_t PCLbits __at(0x002);

# 112
extern volatile unsigned char STATUS __at(0x003);

asm("STATUS equ 03h");


typedef union {
struct {
unsigned C :1;
unsigned DC :1;
unsigned Z :1;
unsigned nPD :1;
unsigned nTO :1;
};
struct {
unsigned CARRY :1;
unsigned :1;
unsigned ZERO :1;
};
} STATUSbits_t;
extern volatile STATUSbits_t STATUSbits __at(0x003);

# 171
extern volatile unsigned short FSR0 __at(0x004);



extern volatile unsigned char FSR0L __at(0x004);

asm("FSR0L equ 04h");


typedef union {
struct {
unsigned FSR0L :8;
};
} FSR0Lbits_t;
extern volatile FSR0Lbits_t FSR0Lbits __at(0x004);

# 195
extern volatile unsigned char FSR0H __at(0x005);

asm("FSR0H equ 05h");


typedef union {
struct {
unsigned FSR0H :8;
};
} FSR0Hbits_t;
extern volatile FSR0Hbits_t FSR0Hbits __at(0x005);

# 215
extern volatile unsigned short FSR1 __at(0x006);



extern volatile unsigned char FSR1L __at(0x006);

asm("FSR1L equ 06h");


typedef union {
struct {
unsigned FSR1L :8;
};
} FSR1Lbits_t;
extern volatile FSR1Lbits_t FSR1Lbits __at(0x006);

# 239
extern volatile unsigned char FSR1H __at(0x007);

asm("FSR1H equ 07h");


typedef union {
struct {
unsigned FSR1H :8;
};
} FSR1Hbits_t;
extern volatile FSR1Hbits_t FSR1Hbits __at(0x007);

# 259
extern volatile unsigned char BSR __at(0x008);

asm("BSR equ 08h");


typedef union {
struct {
unsigned BSR0 :1;
unsigned BSR1 :1;
unsigned BSR2 :1;
unsigned BSR3 :1;
unsigned BSR4 :1;
};
struct {
unsigned BSR :5;
};
} BSRbits_t;
extern volatile BSRbits_t BSRbits __at(0x008);

# 311
extern volatile unsigned char WREG __at(0x009);

asm("WREG equ 09h");


typedef union {
struct {
unsigned WREG0 :8;
};
} WREGbits_t;
extern volatile WREGbits_t WREGbits __at(0x009);

# 331
extern volatile unsigned char PCLATH __at(0x00A);

asm("PCLATH equ 0Ah");


typedef union {
struct {
unsigned PCLATH :7;
};
} PCLATHbits_t;
extern volatile PCLATHbits_t PCLATHbits __at(0x00A);

# 351
extern volatile unsigned char INTCON __at(0x00B);

asm("INTCON equ 0Bh");


typedef union {
struct {
unsigned IOCIF :1;
unsigned INTF :1;
unsigned TMR0IF :1;
unsigned IOCIE :1;
unsigned INTE :1;
unsigned TMR0IE :1;
unsigned PEIE :1;
unsigned GIE :1;
};
struct {
unsigned :2;
unsigned T0IF :1;
unsigned :2;
unsigned T0IE :1;
};
} INTCONbits_t;
extern volatile INTCONbits_t INTCONbits __at(0x00B);

# 429
extern volatile unsigned char PORTA __at(0x00C);

asm("PORTA equ 0Ch");


typedef union {
struct {
unsigned RA0 :1;
unsigned RA1 :1;
unsigned RA2 :1;
unsigned RA3 :1;
unsigned RA4 :1;
unsigned RA5 :1;
};
} PORTAbits_t;
extern volatile PORTAbits_t PORTAbits __at(0x00C);

# 479
extern volatile unsigned char PIR1 __at(0x011);

asm("PIR1 equ 011h");


typedef union {
struct {
unsigned TMR1IF :1;
unsigned TMR2IF :1;
unsigned CCP1IF :1;
unsigned SSP1IF :1;
unsigned TXIF :1;
unsigned RCIF :1;
unsigned ADIF :1;
unsigned TMR1GIF :1;
};
} PIR1bits_t;
extern volatile PIR1bits_t PIR1bits __at(0x011);

# 541
extern volatile unsigned char PIR2 __at(0x012);

asm("PIR2 equ 012h");


typedef union {
struct {
unsigned :3;
unsigned BCL1IF :1;
unsigned EEIF :1;
unsigned C1IF :1;
unsigned :1;
unsigned OSFIF :1;
};
} PIR2bits_t;
extern volatile PIR2bits_t PIR2bits __at(0x012);

# 581
extern volatile unsigned char TMR0 __at(0x015);

asm("TMR0 equ 015h");


typedef union {
struct {
unsigned TMR0 :8;
};
} TMR0bits_t;
extern volatile TMR0bits_t TMR0bits __at(0x015);

# 601
extern volatile unsigned short TMR1 __at(0x016);

asm("TMR1 equ 016h");




extern volatile unsigned char TMR1L __at(0x016);

asm("TMR1L equ 016h");


typedef union {
struct {
unsigned TMR1L :8;
};
} TMR1Lbits_t;
extern volatile TMR1Lbits_t TMR1Lbits __at(0x016);

# 628
extern volatile unsigned char TMR1H __at(0x017);

asm("TMR1H equ 017h");


typedef union {
struct {
unsigned TMR1H :8;
};
} TMR1Hbits_t;
extern volatile TMR1Hbits_t TMR1Hbits __at(0x017);

# 648
extern volatile unsigned char T1CON __at(0x018);

asm("T1CON equ 018h");


typedef union {
struct {
unsigned TMR1ON :1;
unsigned :1;
unsigned nT1SYNC :1;
unsigned T1OSCEN :1;
unsigned T1CKPS0 :1;
unsigned T1CKPS1 :1;
unsigned TMR1CS0 :1;
unsigned TMR1CS1 :1;
};
struct {
unsigned :4;
unsigned T1CKPS :2;
unsigned TMR1CS :2;
};
} T1CONbits_t;
extern volatile T1CONbits_t T1CONbits __at(0x018);

# 720
extern volatile unsigned char T1GCON __at(0x019);

asm("T1GCON equ 019h");


typedef union {
struct {
unsigned T1GSS0 :1;
unsigned T1GSS1 :1;
unsigned T1GVAL :1;
unsigned T1GGO_nDONE :1;
unsigned T1GSPM :1;
unsigned T1GTM :1;
unsigned T1GPOL :1;
unsigned TMR1GE :1;
};
struct {
unsigned T1GSS :2;
unsigned :1;
unsigned T1GGO :1;
};
} T1GCONbits_t;
extern volatile T1GCONbits_t T1GCONbits __at(0x019);

# 797
extern volatile unsigned char TMR2 __at(0x01A);

asm("TMR2 equ 01Ah");


typedef union {
struct {
unsigned TMR2 :8;
};
} TMR2bits_t;
extern volatile TMR2bits_t TMR2bits __at(0x01A);

# 817
extern volatile unsigned char PR2 __at(0x01B);

asm("PR2 equ 01Bh");


typedef union {
struct {
unsigned PR2 :8;
};
} PR2bits_t;
extern volatile PR2bits_t PR2bits __at(0x01B);

# 837
extern volatile unsigned char T2CON __at(0x01C);

asm("T2CON equ 01Ch");


typedef union {
struct {
unsigned T2CKPS0 :1;
unsigned T2CKPS1 :1;
unsigned TMR2ON :1;
unsigned T2OUTPS0 :1;
unsigned T2OUTPS1 :1;
unsigned T2OUTPS2 :1;
unsigned T2OUTPS3 :1;
};
struct {
unsigned T2CKPS :2;
unsigned :1;
unsigned T2OUTPS :4;
};
} T2CONbits_t;
extern volatile T2CONbits_t T2CONbits __at(0x01C);

# 908
extern volatile unsigned char CPSCON0 __at(0x01E);

asm("CPSCON0 equ 01Eh");


typedef union {
struct {
unsigned T0XCS :1;
unsigned CPSOUT :1;
unsigned CPSRNG0 :1;
unsigned CPSRNG1 :1;
unsigned :2;
unsigned CPSRM :1;
unsigned CPSON :1;
};
struct {
unsigned :2;
unsigned CPSRNG :2;
};
} CPSCON0bits_t;
extern volatile CPSCON0bits_t CPSCON0bits __at(0x01E);

# 968
extern volatile unsigned char CPSCON1 __at(0x01F);

asm("CPSCON1 equ 01Fh");


typedef union {
struct {
unsigned CPSCH0 :1;
unsigned CPSCH1 :1;
};
struct {
unsigned CPSCH :2;
};
} CPSCON1bits_t;
extern volatile CPSCON1bits_t CPSCON1bits __at(0x01F);

# 1002
extern volatile unsigned char TRISA __at(0x08C);

asm("TRISA equ 08Ch");


typedef union {
struct {
unsigned TRISA0 :1;
unsigned TRISA1 :1;
unsigned TRISA2 :1;
unsigned TRISA3 :1;
unsigned TRISA4 :1;
unsigned TRISA5 :1;
};
} TRISAbits_t;
extern volatile TRISAbits_t TRISAbits __at(0x08C);

# 1052
extern volatile unsigned char PIE1 __at(0x091);

asm("PIE1 equ 091h");


typedef union {
struct {
unsigned TMR1IE :1;
unsigned TMR2IE :1;
unsigned CCP1IE :1;
unsigned SSP1IE :1;
unsigned TXIE :1;
unsigned RCIE :1;
unsigned ADIE :1;
unsigned TMR1GIE :1;
};
} PIE1bits_t;
extern volatile PIE1bits_t PIE1bits __at(0x091);

# 1114
extern volatile unsigned char PIE2 __at(0x092);

asm("PIE2 equ 092h");


typedef union {
struct {
unsigned :3;
unsigned BCL1IE :1;
unsigned EEIE :1;
unsigned C1IE :1;
unsigned :1;
unsigned OSFIE :1;
};
} PIE2bits_t;
extern volatile PIE2bits_t PIE2bits __at(0x092);

# 1154
extern volatile unsigned char OPTION_REG __at(0x095);

asm("OPTION_REG equ 095h");


typedef union {
struct {
unsigned PS0 :1;
unsigned PS1 :1;
unsigned PS2 :1;
unsigned PSA :1;
unsigned TMR0SE :1;
unsigned TMR0CS :1;
unsigned INTEDG :1;
unsigned nWPUEN :1;
};
struct {
unsigned PS :3;
unsigned :1;
unsigned T0SE :1;
unsigned T0CS :1;
};
} OPTION_REGbits_t;
extern volatile OPTION_REGbits_t OPTION_REGbits __at(0x095);

# 1237
extern volatile unsigned char PCON __at(0x096);

asm("PCON equ 096h");


typedef union {
struct {
unsigned nBOR :1;
unsigned nPOR :1;
unsigned nRI :1;
unsigned nRMCLR :1;
unsigned :2;
unsigned STKUNF :1;
unsigned STKOVF :1;
};
} PCONbits_t;
extern volatile PCONbits_t PCONbits __at(0x096);

# 1288
extern volatile unsigned char WDTCON __at(0x097);

asm("WDTCON equ 097h");


typedef union {
struct {
unsigned SWDTEN :1;
unsigned WDTPS0 :1;
unsigned WDTPS1 :1;
unsigned WDTPS2 :1;
unsigned WDTPS3 :1;
unsigned WDTPS4 :1;
};
struct {
unsigned :1;
unsigned WDTPS :5;
};
} WDTCONbits_t;
extern volatile WDTCONbits_t WDTCONbits __at(0x097);

# 1347
extern volatile unsigned char OSCTUNE __at(0x098);

asm("OSCTUNE equ 098h");


typedef union {
struct {
unsigned TUN0 :1;
unsigned TUN1 :1;
unsigned TUN2 :1;
unsigned TUN3 :1;
unsigned TUN4 :1;
unsigned TUN5 :1;
};
struct {
unsigned TUN :6;
};
} OSCTUNEbits_t;
extern volatile OSCTUNEbits_t OSCTUNEbits __at(0x098);

# 1405
extern volatile unsigned char OSCCON __at(0x099);

asm("OSCCON equ 099h");


typedef union {
struct {
unsigned SCS0 :1;
unsigned SCS1 :1;
unsigned :1;
unsigned IRCF0 :1;
unsigned IRCF1 :1;
unsigned IRCF2 :1;
unsigned IRCF3 :1;
unsigned SPLLEN :1;
};
struct {
unsigned SCS :2;
unsigned :1;
unsigned IRCF :4;
};
} OSCCONbits_t;
extern volatile OSCCONbits_t OSCCONbits __at(0x099);

# 1477
extern volatile unsigned char OSCSTAT __at(0x09A);

asm("OSCSTAT equ 09Ah");


typedef union {
struct {
unsigned HFIOFS :1;
unsigned LFIOFR :1;
unsigned MFIOFR :1;
unsigned HFIOFL :1;
unsigned HFIOFR :1;
unsigned OSTS :1;
unsigned PLLR :1;
unsigned T1OSCR :1;
};
} OSCSTATbits_t;
extern volatile OSCSTATbits_t OSCSTATbits __at(0x09A);

# 1539
extern volatile unsigned short ADRES __at(0x09B);

asm("ADRES equ 09Bh");




extern volatile unsigned char ADRESL __at(0x09B);

asm("ADRESL equ 09Bh");


typedef union {
struct {
unsigned ADRESL :8;
};
} ADRESLbits_t;
extern volatile ADRESLbits_t ADRESLbits __at(0x09B);

# 1566
extern volatile unsigned char ADRESH __at(0x09C);

asm("ADRESH equ 09Ch");


typedef union {
struct {
unsigned ADRESH :8;
};
} ADRESHbits_t;
extern volatile ADRESHbits_t ADRESHbits __at(0x09C);

# 1586
extern volatile unsigned char ADCON0 __at(0x09D);

asm("ADCON0 equ 09Dh");


typedef union {
struct {
unsigned ADON :1;
unsigned GO_nDONE :1;
unsigned CHS0 :1;
unsigned CHS1 :1;
unsigned CHS2 :1;
unsigned CHS3 :1;
unsigned CHS4 :1;
};
struct {
unsigned :1;
unsigned ADGO :1;
unsigned CHS :5;
};
struct {
unsigned :1;
unsigned GO :1;
};
} ADCON0bits_t;
extern volatile ADCON0bits_t ADCON0bits __at(0x09D);

# 1666
extern volatile unsigned char ADCON1 __at(0x09E);

asm("ADCON1 equ 09Eh");


typedef union {
struct {
unsigned ADPREF0 :1;
unsigned ADPREF1 :1;
unsigned :2;
unsigned ADCS0 :1;
unsigned ADCS1 :1;
unsigned ADCS2 :1;
unsigned ADFM :1;
};
struct {
unsigned ADPREF :2;
unsigned :2;
unsigned ADCS :3;
};
} ADCON1bits_t;
extern volatile ADCON1bits_t ADCON1bits __at(0x09E);

# 1732
extern volatile unsigned char LATA __at(0x10C);

asm("LATA equ 010Ch");


typedef union {
struct {
unsigned LATA0 :1;
unsigned LATA1 :1;
unsigned LATA2 :1;
unsigned :1;
unsigned LATA4 :1;
unsigned LATA5 :1;
};
} LATAbits_t;
extern volatile LATAbits_t LATAbits __at(0x10C);

# 1777
extern volatile unsigned char CM1CON0 __at(0x111);

asm("CM1CON0 equ 0111h");


typedef union {
struct {
unsigned C1SYNC :1;
unsigned C1HYS :1;
unsigned C1SP :1;
unsigned :1;
unsigned C1POL :1;
unsigned C1OE :1;
unsigned C1OUT :1;
unsigned C1ON :1;
};
} CM1CON0bits_t;
extern volatile CM1CON0bits_t CM1CON0bits __at(0x111);

# 1834
extern volatile unsigned char CM1CON1 __at(0x112);

asm("CM1CON1 equ 0112h");


typedef union {
struct {
unsigned C1NCH :1;
unsigned :3;
unsigned C1PCH0 :1;
unsigned C1PCH1 :1;
unsigned C1INTN :1;
unsigned C1INTP :1;
};
struct {
unsigned C1NCH0 :1;
unsigned :3;
unsigned C1PCH :2;
};
} CM1CON1bits_t;
extern volatile CM1CON1bits_t CM1CON1bits __at(0x112);

# 1894
extern volatile unsigned char CMOUT __at(0x115);

asm("CMOUT equ 0115h");


typedef union {
struct {
unsigned MC1OUT :1;
};
} CMOUTbits_t;
extern volatile CMOUTbits_t CMOUTbits __at(0x115);

# 1914
extern volatile unsigned char BORCON __at(0x116);

asm("BORCON equ 0116h");


typedef union {
struct {
unsigned BORRDY :1;
unsigned :5;
unsigned BORFS :1;
unsigned SBOREN :1;
};
} BORCONbits_t;
extern volatile BORCONbits_t BORCONbits __at(0x116);

# 1947
extern volatile unsigned char FVRCON __at(0x117);

asm("FVRCON equ 0117h");


typedef union {
struct {
unsigned ADFVR0 :1;
unsigned ADFVR1 :1;
unsigned CDAFVR0 :1;
unsigned CDAFVR1 :1;
unsigned TSRNG :1;
unsigned TSEN :1;
unsigned FVRRDY :1;
unsigned FVREN :1;
};
struct {
unsigned ADFVR :2;
unsigned CDAFVR :2;
};
} FVRCONbits_t;
extern volatile FVRCONbits_t FVRCONbits __at(0x117);

# 2023
extern volatile unsigned char DACCON0 __at(0x118);

asm("DACCON0 equ 0118h");


typedef union {
struct {
unsigned :2;
unsigned DACPSS0 :1;
unsigned DACPSS1 :1;
unsigned :1;
unsigned DACOE :1;
unsigned DACLPS :1;
unsigned DACEN :1;
};
struct {
unsigned :2;
unsigned DACPSS :2;
};
} DACCON0bits_t;
extern volatile DACCON0bits_t DACCON0bits __at(0x118);

# 2078
extern volatile unsigned char DACCON1 __at(0x119);

asm("DACCON1 equ 0119h");


typedef union {
struct {
unsigned DACR0 :1;
unsigned DACR1 :1;
unsigned DACR2 :1;
unsigned DACR3 :1;
unsigned DACR4 :1;
};
struct {
unsigned DACR :5;
};
} DACCON1bits_t;
extern volatile DACCON1bits_t DACCON1bits __at(0x119);

# 2130
extern volatile unsigned char SRCON0 __at(0x11A);

asm("SRCON0 equ 011Ah");


typedef union {
struct {
unsigned SRPR :1;
unsigned SRPS :1;
unsigned SRNQEN :1;
unsigned SRQEN :1;
unsigned SRCLK0 :1;
unsigned SRCLK1 :1;
unsigned SRCLK2 :1;
unsigned SRLEN :1;
};
struct {
unsigned :4;
unsigned SRCLK :3;
};
} SRCON0bits_t;
extern volatile SRCON0bits_t SRCON0bits __at(0x11A);

# 2201
extern volatile unsigned char SRCON1 __at(0x11B);

asm("SRCON1 equ 011Bh");


typedef union {
struct {
unsigned SRRC1E :1;
unsigned :1;
unsigned SRRCKE :1;
unsigned SRRPE :1;
unsigned SRSC1E :1;
unsigned :1;
unsigned SRSCKE :1;
unsigned SRSPE :1;
};
} SRCON1bits_t;
extern volatile SRCON1bits_t SRCON1bits __at(0x11B);

# 2253
extern volatile unsigned char APFCON __at(0x11D);

asm("APFCON equ 011Dh");


extern volatile unsigned char APFCON0 __at(0x11D);

asm("APFCON0 equ 011Dh");


typedef union {
struct {
unsigned CCP1SEL :1;
unsigned P1BSEL :1;
unsigned TXCKSEL :1;
unsigned T1GSEL :1;
unsigned :1;
unsigned SSSEL :1;
unsigned SDOSEL :1;
unsigned RXDTSEL :1;
};
struct {
unsigned :5;
unsigned SS1SEL :1;
unsigned SDO1SEL :1;
};
} APFCONbits_t;
extern volatile APFCONbits_t APFCONbits __at(0x11D);

# 2328
typedef union {
struct {
unsigned CCP1SEL :1;
unsigned P1BSEL :1;
unsigned TXCKSEL :1;
unsigned T1GSEL :1;
unsigned :1;
unsigned SSSEL :1;
unsigned SDOSEL :1;
unsigned RXDTSEL :1;
};
struct {
unsigned :5;
unsigned SS1SEL :1;
unsigned SDO1SEL :1;
};
} APFCON0bits_t;
extern volatile APFCON0bits_t APFCON0bits __at(0x11D);

# 2395
extern volatile unsigned char ANSELA __at(0x18C);

asm("ANSELA equ 018Ch");


typedef union {
struct {
unsigned ANSA0 :1;
unsigned ANSA1 :1;
unsigned ANSA2 :1;
unsigned :1;
unsigned ANSA4 :1;
};
struct {
unsigned ANSELA :5;
};
} ANSELAbits_t;
extern volatile ANSELAbits_t ANSELAbits __at(0x18C);

# 2442
extern volatile unsigned short EEADR __at(0x191);

asm("EEADR equ 0191h");




extern volatile unsigned char EEADRL __at(0x191);

asm("EEADRL equ 0191h");


typedef union {
struct {
unsigned EEADRL :8;
};
} EEADRLbits_t;
extern volatile EEADRLbits_t EEADRLbits __at(0x191);

# 2469
extern volatile unsigned char EEADRH __at(0x192);

asm("EEADRH equ 0192h");


typedef union {
struct {
unsigned EEADRH :7;
};
} EEADRHbits_t;
extern volatile EEADRHbits_t EEADRHbits __at(0x192);

# 2489
extern volatile unsigned short EEDAT __at(0x193);

asm("EEDAT equ 0193h");




extern volatile unsigned char EEDATL __at(0x193);

asm("EEDATL equ 0193h");


extern volatile unsigned char EEDATA __at(0x193);

asm("EEDATA equ 0193h");


typedef union {
struct {
unsigned EEDATL :8;
};
} EEDATLbits_t;
extern volatile EEDATLbits_t EEDATLbits __at(0x193);

# 2519
typedef union {
struct {
unsigned EEDATL :8;
};
} EEDATAbits_t;
extern volatile EEDATAbits_t EEDATAbits __at(0x193);

# 2534
extern volatile unsigned char EEDATH __at(0x194);

asm("EEDATH equ 0194h");


typedef union {
struct {
unsigned EEDATH :6;
};
} EEDATHbits_t;
extern volatile EEDATHbits_t EEDATHbits __at(0x194);

# 2554
extern volatile unsigned char EECON1 __at(0x195);

asm("EECON1 equ 0195h");


typedef union {
struct {
unsigned RD :1;
unsigned WR :1;
unsigned WREN :1;
unsigned WRERR :1;
unsigned FREE :1;
unsigned LWLO :1;
unsigned CFGS :1;
unsigned EEPGD :1;
};
} EECON1bits_t;
extern volatile EECON1bits_t EECON1bits __at(0x195);

# 2616
extern volatile unsigned char EECON2 __at(0x196);

asm("EECON2 equ 0196h");


typedef union {
struct {
unsigned EECON2 :8;
};
} EECON2bits_t;
extern volatile EECON2bits_t EECON2bits __at(0x196);

# 2636
extern volatile unsigned char VREGCON __at(0x197);

asm("VREGCON equ 0197h");


typedef union {
struct {
unsigned VREGPM0 :1;
unsigned VREGPM1 :1;
};
struct {
unsigned VREGPM :2;
};
} VREGCONbits_t;
extern volatile VREGCONbits_t VREGCONbits __at(0x197);

# 2670
extern volatile unsigned char RCREG __at(0x199);

asm("RCREG equ 0199h");


typedef union {
struct {
unsigned RCREG :8;
};
} RCREGbits_t;
extern volatile RCREGbits_t RCREGbits __at(0x199);

# 2690
extern volatile unsigned char TXREG __at(0x19A);

asm("TXREG equ 019Ah");


typedef union {
struct {
unsigned TXREG :8;
};
} TXREGbits_t;
extern volatile TXREGbits_t TXREGbits __at(0x19A);

# 2710
extern volatile unsigned short SP1BRG __at(0x19B);

asm("SP1BRG equ 019Bh");




extern volatile unsigned char SP1BRGL __at(0x19B);

asm("SP1BRGL equ 019Bh");


extern volatile unsigned char SPBRG __at(0x19B);

asm("SPBRG equ 019Bh");

extern volatile unsigned char SPBRGL __at(0x19B);

asm("SPBRGL equ 019Bh");


typedef union {
struct {
unsigned SPBRGL :8;
};
} SP1BRGLbits_t;
extern volatile SP1BRGLbits_t SP1BRGLbits __at(0x19B);

# 2744
typedef union {
struct {
unsigned SPBRGL :8;
};
} SPBRGbits_t;
extern volatile SPBRGbits_t SPBRGbits __at(0x19B);

# 2756
typedef union {
struct {
unsigned SPBRGL :8;
};
} SPBRGLbits_t;
extern volatile SPBRGLbits_t SPBRGLbits __at(0x19B);

# 2771
extern volatile unsigned char SP1BRGH __at(0x19C);

asm("SP1BRGH equ 019Ch");


extern volatile unsigned char SPBRGH __at(0x19C);

asm("SPBRGH equ 019Ch");


typedef union {
struct {
unsigned SPBRGH :8;
};
} SP1BRGHbits_t;
extern volatile SP1BRGHbits_t SP1BRGHbits __at(0x19C);

# 2794
typedef union {
struct {
unsigned SPBRGH :8;
};
} SPBRGHbits_t;
extern volatile SPBRGHbits_t SPBRGHbits __at(0x19C);

# 2809
extern volatile unsigned char RCSTA __at(0x19D);

asm("RCSTA equ 019Dh");


typedef union {
struct {
unsigned RX9D :1;
unsigned OERR :1;
unsigned FERR :1;
unsigned ADDEN :1;
unsigned CREN :1;
unsigned SREN :1;
unsigned RX9 :1;
unsigned SPEN :1;
};
} RCSTAbits_t;
extern volatile RCSTAbits_t RCSTAbits __at(0x19D);

# 2871
extern volatile unsigned char TXSTA __at(0x19E);

asm("TXSTA equ 019Eh");


typedef union {
struct {
unsigned TX9D :1;
unsigned TRMT :1;
unsigned BRGH :1;
unsigned SENDB :1;
unsigned SYNC :1;
unsigned TXEN :1;
unsigned TX9 :1;
unsigned CSRC :1;
};
} TXSTAbits_t;
extern volatile TXSTAbits_t TXSTAbits __at(0x19E);

# 2933
extern volatile unsigned char BAUDCON __at(0x19F);

asm("BAUDCON equ 019Fh");


typedef union {
struct {
unsigned ABDEN :1;
unsigned WUE :1;
unsigned :1;
unsigned BRG16 :1;
unsigned SCKP :1;
unsigned :1;
unsigned RCIDL :1;
unsigned ABDOVF :1;
};
} BAUDCONbits_t;
extern volatile BAUDCONbits_t BAUDCONbits __at(0x19F);

# 2985
extern volatile unsigned char WPUA __at(0x20C);

asm("WPUA equ 020Ch");


typedef union {
struct {
unsigned WPUA0 :1;
unsigned WPUA1 :1;
unsigned WPUA2 :1;
unsigned WPUA3 :1;
unsigned WPUA4 :1;
unsigned WPUA5 :1;
};
struct {
unsigned WPUA :6;
};
} WPUAbits_t;
extern volatile WPUAbits_t WPUAbits __at(0x20C);

# 3043
extern volatile unsigned char SSP1BUF __at(0x211);

asm("SSP1BUF equ 0211h");


extern volatile unsigned char SSPBUF __at(0x211);

asm("SSPBUF equ 0211h");


typedef union {
struct {
unsigned SSPBUF :8;
};
} SSP1BUFbits_t;
extern volatile SSP1BUFbits_t SSP1BUFbits __at(0x211);

# 3066
typedef union {
struct {
unsigned SSPBUF :8;
};
} SSPBUFbits_t;
extern volatile SSPBUFbits_t SSPBUFbits __at(0x211);

# 3081
extern volatile unsigned char SSP1ADD __at(0x212);

asm("SSP1ADD equ 0212h");


extern volatile unsigned char SSPADD __at(0x212);

asm("SSPADD equ 0212h");


typedef union {
struct {
unsigned SSPADD :8;
};
} SSP1ADDbits_t;
extern volatile SSP1ADDbits_t SSP1ADDbits __at(0x212);

# 3104
typedef union {
struct {
unsigned SSPADD :8;
};
} SSPADDbits_t;
extern volatile SSPADDbits_t SSPADDbits __at(0x212);

# 3119
extern volatile unsigned char SSP1MSK __at(0x213);

asm("SSP1MSK equ 0213h");


extern volatile unsigned char SSPMSK __at(0x213);

asm("SSPMSK equ 0213h");


typedef union {
struct {
unsigned SSPMSK :8;
};
} SSP1MSKbits_t;
extern volatile SSP1MSKbits_t SSP1MSKbits __at(0x213);

# 3142
typedef union {
struct {
unsigned SSPMSK :8;
};
} SSPMSKbits_t;
extern volatile SSPMSKbits_t SSPMSKbits __at(0x213);

# 3157
extern volatile unsigned char SSP1STAT __at(0x214);

asm("SSP1STAT equ 0214h");


extern volatile unsigned char SSPSTAT __at(0x214);

asm("SSPSTAT equ 0214h");


typedef union {
struct {
unsigned BF :1;
unsigned UA :1;
unsigned R_nW :1;
unsigned S :1;
unsigned P :1;
unsigned D_nA :1;
unsigned CKE :1;
unsigned SMP :1;
};
} SSP1STATbits_t;
extern volatile SSP1STATbits_t SSP1STATbits __at(0x214);

# 3222
typedef union {
struct {
unsigned BF :1;
unsigned UA :1;
unsigned R_nW :1;
unsigned S :1;
unsigned P :1;
unsigned D_nA :1;
unsigned CKE :1;
unsigned SMP :1;
};
} SSPSTATbits_t;
extern volatile SSPSTATbits_t SSPSTATbits __at(0x214);

# 3279
extern volatile unsigned char SSP1CON1 __at(0x215);

asm("SSP1CON1 equ 0215h");


extern volatile unsigned char SSPCON1 __at(0x215);

asm("SSPCON1 equ 0215h");

extern volatile unsigned char SSPCON __at(0x215);

asm("SSPCON equ 0215h");


typedef union {
struct {
unsigned SSPM0 :1;
unsigned SSPM1 :1;
unsigned SSPM2 :1;
unsigned SSPM3 :1;
unsigned CKP :1;
unsigned SSPEN :1;
unsigned SSPOV :1;
unsigned WCOL :1;
};
struct {
unsigned SSPM :4;
};
} SSP1CON1bits_t;
extern volatile SSP1CON1bits_t SSP1CON1bits __at(0x215);

# 3356
typedef union {
struct {
unsigned SSPM0 :1;
unsigned SSPM1 :1;
unsigned SSPM2 :1;
unsigned SSPM3 :1;
unsigned CKP :1;
unsigned SSPEN :1;
unsigned SSPOV :1;
unsigned WCOL :1;
};
struct {
unsigned SSPM :4;
};
} SSPCON1bits_t;
extern volatile SSPCON1bits_t SSPCON1bits __at(0x215);

# 3418
typedef union {
struct {
unsigned SSPM0 :1;
unsigned SSPM1 :1;
unsigned SSPM2 :1;
unsigned SSPM3 :1;
unsigned CKP :1;
unsigned SSPEN :1;
unsigned SSPOV :1;
unsigned WCOL :1;
};
struct {
unsigned SSPM :4;
};
} SSPCONbits_t;
extern volatile SSPCONbits_t SSPCONbits __at(0x215);

# 3483
extern volatile unsigned char SSP1CON2 __at(0x216);

asm("SSP1CON2 equ 0216h");


extern volatile unsigned char SSPCON2 __at(0x216);

asm("SSPCON2 equ 0216h");


typedef union {
struct {
unsigned SEN :1;
unsigned RSEN :1;
unsigned PEN :1;
unsigned RCEN :1;
unsigned ACKEN :1;
unsigned ACKDT :1;
unsigned ACKSTAT :1;
unsigned GCEN :1;
};
} SSP1CON2bits_t;
extern volatile SSP1CON2bits_t SSP1CON2bits __at(0x216);

# 3548
typedef union {
struct {
unsigned SEN :1;
unsigned RSEN :1;
unsigned PEN :1;
unsigned RCEN :1;
unsigned ACKEN :1;
unsigned ACKDT :1;
unsigned ACKSTAT :1;
unsigned GCEN :1;
};
} SSPCON2bits_t;
extern volatile SSPCON2bits_t SSPCON2bits __at(0x216);

# 3605
extern volatile unsigned char SSP1CON3 __at(0x217);

asm("SSP1CON3 equ 0217h");


extern volatile unsigned char SSPCON3 __at(0x217);

asm("SSPCON3 equ 0217h");


typedef union {
struct {
unsigned DHEN :1;
unsigned AHEN :1;
unsigned SBCDE :1;
unsigned SDAHT :1;
unsigned BOEN :1;
unsigned SCIE :1;
unsigned PCIE :1;
unsigned ACKTIM :1;
};
} SSP1CON3bits_t;
extern volatile SSP1CON3bits_t SSP1CON3bits __at(0x217);

# 3670
typedef union {
struct {
unsigned DHEN :1;
unsigned AHEN :1;
unsigned SBCDE :1;
unsigned SDAHT :1;
unsigned BOEN :1;
unsigned SCIE :1;
unsigned PCIE :1;
unsigned ACKTIM :1;
};
} SSPCON3bits_t;
extern volatile SSPCON3bits_t SSPCON3bits __at(0x217);

# 3727
extern volatile unsigned short CCPR1 __at(0x291);

asm("CCPR1 equ 0291h");




extern volatile unsigned char CCPR1L __at(0x291);

asm("CCPR1L equ 0291h");


typedef union {
struct {
unsigned CCPR1L :8;
};
} CCPR1Lbits_t;
extern volatile CCPR1Lbits_t CCPR1Lbits __at(0x291);

# 3754
extern volatile unsigned char CCPR1H __at(0x292);

asm("CCPR1H equ 0292h");


typedef union {
struct {
unsigned CCPR1H :8;
};
} CCPR1Hbits_t;
extern volatile CCPR1Hbits_t CCPR1Hbits __at(0x292);

# 3774
extern volatile unsigned char CCP1CON __at(0x293);

asm("CCP1CON equ 0293h");


typedef union {
struct {
unsigned CCP1M0 :1;
unsigned CCP1M1 :1;
unsigned CCP1M2 :1;
unsigned CCP1M3 :1;
unsigned DC1B0 :1;
unsigned DC1B1 :1;
unsigned P1M0 :1;
unsigned P1M1 :1;
};
struct {
unsigned CCP1M :4;
unsigned DC1B :2;
unsigned P1M :2;
};
} CCP1CONbits_t;
extern volatile CCP1CONbits_t CCP1CONbits __at(0x293);

# 3856
extern volatile unsigned char PWM1CON __at(0x294);

asm("PWM1CON equ 0294h");


typedef union {
struct {
unsigned P1DC0 :1;
unsigned P1DC1 :1;
unsigned P1DC2 :1;
unsigned P1DC3 :1;
unsigned P1DC4 :1;
unsigned P1DC5 :1;
unsigned P1DC6 :1;
unsigned P1RSEN :1;
};
struct {
unsigned P1DC :7;
};
} PWM1CONbits_t;
extern volatile PWM1CONbits_t PWM1CONbits __at(0x294);

# 3926
extern volatile unsigned char CCP1AS __at(0x295);

asm("CCP1AS equ 0295h");


extern volatile unsigned char ECCP1AS __at(0x295);

asm("ECCP1AS equ 0295h");


typedef union {
struct {
unsigned PSS1BD0 :1;
unsigned PSS1BD1 :1;
unsigned PSS1AC0 :1;
unsigned PSS1AC1 :1;
unsigned CCP1AS0 :1;
unsigned CCP1AS1 :1;
unsigned CCP1AS2 :1;
unsigned CCP1ASE :1;
};
struct {
unsigned PSS1BD :2;
unsigned PSS1AC :2;
unsigned CCP1AS :3;
};
} CCP1ASbits_t;
extern volatile CCP1ASbits_t CCP1ASbits __at(0x295);

# 4011
typedef union {
struct {
unsigned PSS1BD0 :1;
unsigned PSS1BD1 :1;
unsigned PSS1AC0 :1;
unsigned PSS1AC1 :1;
unsigned CCP1AS0 :1;
unsigned CCP1AS1 :1;
unsigned CCP1AS2 :1;
unsigned CCP1ASE :1;
};
struct {
unsigned PSS1BD :2;
unsigned PSS1AC :2;
unsigned CCP1AS :3;
};
} ECCP1ASbits_t;
extern volatile ECCP1ASbits_t ECCP1ASbits __at(0x295);

# 4088
extern volatile unsigned char PSTR1CON __at(0x296);

asm("PSTR1CON equ 0296h");


typedef union {
struct {
unsigned STR1A :1;
unsigned STR1B :1;
unsigned :1;
unsigned :1;
unsigned STR1SYNC :1;
};
} PSTR1CONbits_t;
extern volatile PSTR1CONbits_t PSTR1CONbits __at(0x296);

# 4122
extern volatile unsigned char IOCAP __at(0x391);

asm("IOCAP equ 0391h");


typedef union {
struct {
unsigned IOCAP0 :1;
unsigned IOCAP1 :1;
unsigned IOCAP2 :1;
unsigned IOCAP3 :1;
unsigned IOCAP4 :1;
unsigned IOCAP5 :1;
};
struct {
unsigned IOCAP :6;
};
} IOCAPbits_t;
extern volatile IOCAPbits_t IOCAPbits __at(0x391);

# 4180
extern volatile unsigned char IOCAN __at(0x392);

asm("IOCAN equ 0392h");


typedef union {
struct {
unsigned IOCAN0 :1;
unsigned IOCAN1 :1;
unsigned IOCAN2 :1;
unsigned IOCAN3 :1;
unsigned IOCAN4 :1;
unsigned IOCAN5 :1;
};
struct {
unsigned IOCAN :6;
};
} IOCANbits_t;
extern volatile IOCANbits_t IOCANbits __at(0x392);

# 4238
extern volatile unsigned char IOCAF __at(0x393);

asm("IOCAF equ 0393h");


typedef union {
struct {
unsigned IOCAF0 :1;
unsigned IOCAF1 :1;
unsigned IOCAF2 :1;
unsigned IOCAF3 :1;
unsigned IOCAF4 :1;
unsigned IOCAF5 :1;
};
struct {
unsigned IOCAF :6;
};
} IOCAFbits_t;
extern volatile IOCAFbits_t IOCAFbits __at(0x393);

# 4296
extern volatile unsigned char CLKRCON __at(0x39A);

asm("CLKRCON equ 039Ah");


typedef union {
struct {
unsigned CLKRDIV0 :1;
unsigned CLKRDIV1 :1;
unsigned CLKRDIV2 :1;
unsigned CLKRDC0 :1;
unsigned CLKRDC1 :1;
unsigned CLKRSLR :1;
unsigned CLKROE :1;
unsigned CLKREN :1;
};
struct {
unsigned CLKRDIV :3;
unsigned CLKRDC :2;
};
} CLKRCONbits_t;
extern volatile CLKRCONbits_t CLKRCONbits __at(0x39A);

# 4372
extern volatile unsigned char MDCON __at(0x39C);

asm("MDCON equ 039Ch");


typedef union {
struct {
unsigned MDBIT :1;
unsigned :2;
unsigned MDOUT :1;
unsigned MDOPOL :1;
unsigned MDSLR :1;
unsigned MDOE :1;
unsigned MDEN :1;
};
} MDCONbits_t;
extern volatile MDCONbits_t MDCONbits __at(0x39C);

# 4423
extern volatile unsigned char MDSRC __at(0x39D);

asm("MDSRC equ 039Dh");


typedef union {
struct {
unsigned MDMS0 :1;
unsigned MDMS1 :1;
unsigned MDMS2 :1;
unsigned MDMS3 :1;
unsigned :3;
unsigned MDMSODIS :1;
};
struct {
unsigned MDMS :4;
};
} MDSRCbits_t;
extern volatile MDSRCbits_t MDSRCbits __at(0x39D);

# 4476
extern volatile unsigned char MDCARL __at(0x39E);

asm("MDCARL equ 039Eh");


typedef union {
struct {
unsigned MDCL0 :1;
unsigned MDCL1 :1;
unsigned MDCL2 :1;
unsigned MDCL3 :1;
unsigned :1;
unsigned MDCLSYNC :1;
unsigned MDCLPOL :1;
unsigned MDCLODIS :1;
};
struct {
unsigned MDCL :4;
};
} MDCARLbits_t;
extern volatile MDCARLbits_t MDCARLbits __at(0x39E);

# 4541
extern volatile unsigned char MDCARH __at(0x39F);

asm("MDCARH equ 039Fh");


typedef union {
struct {
unsigned MDCH0 :1;
unsigned MDCH1 :1;
unsigned MDCH2 :1;
unsigned MDCH3 :1;
unsigned :1;
unsigned MDCHSYNC :1;
unsigned MDCHPOL :1;
unsigned MDCHODIS :1;
};
struct {
unsigned MDCH :4;
};
} MDCARHbits_t;
extern volatile MDCARHbits_t MDCARHbits __at(0x39F);

# 4606
extern volatile unsigned char STATUS_SHAD __at(0xFE4);

asm("STATUS_SHAD equ 0FE4h");


typedef union {
struct {
unsigned C_SHAD :1;
unsigned DC_SHAD :1;
unsigned Z_SHAD :1;
};
} STATUS_SHADbits_t;
extern volatile STATUS_SHADbits_t STATUS_SHADbits __at(0xFE4);

# 4638
extern volatile unsigned char WREG_SHAD __at(0xFE5);

asm("WREG_SHAD equ 0FE5h");


typedef union {
struct {
unsigned WREG_SHAD :8;
};
} WREG_SHADbits_t;
extern volatile WREG_SHADbits_t WREG_SHADbits __at(0xFE5);

# 4658
extern volatile unsigned char BSR_SHAD __at(0xFE6);

asm("BSR_SHAD equ 0FE6h");


typedef union {
struct {
unsigned BSR_SHAD :5;
};
} BSR_SHADbits_t;
extern volatile BSR_SHADbits_t BSR_SHADbits __at(0xFE6);

# 4678
extern volatile unsigned char PCLATH_SHAD __at(0xFE7);

asm("PCLATH_SHAD equ 0FE7h");


typedef union {
struct {
unsigned PCLATH_SHAD :7;
};
} PCLATH_SHADbits_t;
extern volatile PCLATH_SHADbits_t PCLATH_SHADbits __at(0xFE7);

# 4698
extern volatile unsigned char FSR0L_SHAD __at(0xFE8);

asm("FSR0L_SHAD equ 0FE8h");


typedef union {
struct {
unsigned FSR0L_SHAD :8;
};
} FSR0L_SHADbits_t;
extern volatile FSR0L_SHADbits_t FSR0L_SHADbits __at(0xFE8);

# 4718
extern volatile unsigned char FSR0H_SHAD __at(0xFE9);

asm("FSR0H_SHAD equ 0FE9h");


typedef union {
struct {
unsigned FSR0H_SHAD :8;
};
} FSR0H_SHADbits_t;
extern volatile FSR0H_SHADbits_t FSR0H_SHADbits __at(0xFE9);

# 4738
extern volatile unsigned char FSR1L_SHAD __at(0xFEA);

asm("FSR1L_SHAD equ 0FEAh");


typedef union {
struct {
unsigned FSR1L_SHAD :8;
};
} FSR1L_SHADbits_t;
extern volatile FSR1L_SHADbits_t FSR1L_SHADbits __at(0xFEA);

# 4758
extern volatile unsigned char FSR1H_SHAD __at(0xFEB);

asm("FSR1H_SHAD equ 0FEBh");


typedef union {
struct {
unsigned FSR1H_SHAD :8;
};
} FSR1H_SHADbits_t;
extern volatile FSR1H_SHADbits_t FSR1H_SHADbits __at(0xFEB);

# 4778
extern volatile unsigned char STKPTR __at(0xFED);

asm("STKPTR equ 0FEDh");


typedef union {
struct {
unsigned STKPTR :5;
};
} STKPTRbits_t;
extern volatile STKPTRbits_t STKPTRbits __at(0xFED);

# 4798
extern volatile unsigned char TOSL __at(0xFEE);

asm("TOSL equ 0FEEh");


typedef union {
struct {
unsigned TOSL :8;
};
} TOSLbits_t;
extern volatile TOSLbits_t TOSLbits __at(0xFEE);

# 4818
extern volatile unsigned char TOSH __at(0xFEF);

asm("TOSH equ 0FEFh");


typedef union {
struct {
unsigned TOSH :7;
};
} TOSHbits_t;
extern volatile TOSHbits_t TOSHbits __at(0xFEF);

# 4844
extern volatile __bit ABDEN __at(0xCF8);


extern volatile __bit ABDOVF __at(0xCFF);


extern volatile __bit ACKDT __at(0x10B5);


extern volatile __bit ACKEN __at(0x10B4);


extern volatile __bit ACKSTAT __at(0x10B6);


extern volatile __bit ACKTIM __at(0x10BF);


extern volatile __bit ADCS0 __at(0x4F4);


extern volatile __bit ADCS1 __at(0x4F5);


extern volatile __bit ADCS2 __at(0x4F6);


extern volatile __bit ADDEN __at(0xCEB);


extern volatile __bit ADFM __at(0x4F7);


extern volatile __bit ADFVR0 __at(0x8B8);


extern volatile __bit ADFVR1 __at(0x8B9);


extern volatile __bit ADGO __at(0x4E9);


extern volatile __bit ADIE __at(0x48E);


extern volatile __bit ADIF __at(0x8E);


extern volatile __bit ADON __at(0x4E8);


extern volatile __bit ADPREF0 __at(0x4F0);


extern volatile __bit ADPREF1 __at(0x4F1);


extern volatile __bit AHEN __at(0x10B9);


extern volatile __bit ANSA0 __at(0xC60);


extern volatile __bit ANSA1 __at(0xC61);


extern volatile __bit ANSA2 __at(0xC62);


extern volatile __bit ANSA4 __at(0xC64);


extern volatile __bit BCL1IE __at(0x493);


extern volatile __bit BCL1IF __at(0x93);


extern volatile __bit BF __at(0x10A0);


extern volatile __bit BOEN __at(0x10BC);


extern volatile __bit BORFS __at(0x8B6);


extern volatile __bit BORRDY __at(0x8B0);


extern volatile __bit BRG16 __at(0xCFB);


extern volatile __bit BRGH __at(0xCF2);


extern volatile __bit BSR0 __at(0x40);


extern volatile __bit BSR1 __at(0x41);


extern volatile __bit BSR2 __at(0x42);


extern volatile __bit BSR3 __at(0x43);


extern volatile __bit BSR4 __at(0x44);


extern volatile __bit C1HYS __at(0x889);


extern volatile __bit C1IE __at(0x495);


extern volatile __bit C1IF __at(0x95);


extern volatile __bit C1INTN __at(0x896);


extern volatile __bit C1INTP __at(0x897);


extern volatile __bit C1NCH __at(0x890);


extern volatile __bit C1NCH0 __at(0x890);


extern volatile __bit C1OE __at(0x88D);


extern volatile __bit C1ON __at(0x88F);


extern volatile __bit C1OUT __at(0x88E);


extern volatile __bit C1PCH0 __at(0x894);


extern volatile __bit C1PCH1 __at(0x895);


extern volatile __bit C1POL __at(0x88C);


extern volatile __bit C1SP __at(0x88A);


extern volatile __bit C1SYNC __at(0x888);


extern volatile __bit CARRY __at(0x18);


extern volatile __bit CCP1AS0 __at(0x14AC);


extern volatile __bit CCP1AS1 __at(0x14AD);


extern volatile __bit CCP1AS2 __at(0x14AE);


extern volatile __bit CCP1ASE __at(0x14AF);


extern volatile __bit CCP1IE __at(0x48A);


extern volatile __bit CCP1IF __at(0x8A);


extern volatile __bit CCP1M0 __at(0x1498);


extern volatile __bit CCP1M1 __at(0x1499);


extern volatile __bit CCP1M2 __at(0x149A);


extern volatile __bit CCP1M3 __at(0x149B);


extern volatile __bit CCP1SEL __at(0x8E8);


extern volatile __bit CDAFVR0 __at(0x8BA);


extern volatile __bit CDAFVR1 __at(0x8BB);


extern volatile __bit CFGS __at(0xCAE);


extern volatile __bit CHS0 __at(0x4EA);


extern volatile __bit CHS1 __at(0x4EB);


extern volatile __bit CHS2 __at(0x4EC);


extern volatile __bit CHS3 __at(0x4ED);


extern volatile __bit CHS4 __at(0x4EE);


extern volatile __bit CKE __at(0x10A6);


extern volatile __bit CKP __at(0x10AC);


extern volatile __bit CLKRDC0 __at(0x1CD3);


extern volatile __bit CLKRDC1 __at(0x1CD4);


extern volatile __bit CLKRDIV0 __at(0x1CD0);


extern volatile __bit CLKRDIV1 __at(0x1CD1);


extern volatile __bit CLKRDIV2 __at(0x1CD2);


extern volatile __bit CLKREN __at(0x1CD7);


extern volatile __bit CLKROE __at(0x1CD6);


extern volatile __bit CLKRSLR __at(0x1CD5);


extern volatile __bit CPSCH0 __at(0xF8);


extern volatile __bit CPSCH1 __at(0xF9);


extern volatile __bit CPSON __at(0xF7);


extern volatile __bit CPSOUT __at(0xF1);


extern volatile __bit CPSRM __at(0xF6);


extern volatile __bit CPSRNG0 __at(0xF2);


extern volatile __bit CPSRNG1 __at(0xF3);


extern volatile __bit CREN __at(0xCEC);


extern volatile __bit CSRC __at(0xCF7);


extern volatile __bit C_SHAD __at(0x7F20);


extern volatile __bit DACEN __at(0x8C7);


extern volatile __bit DACLPS __at(0x8C6);


extern volatile __bit DACOE __at(0x8C5);


extern volatile __bit DACPSS0 __at(0x8C2);


extern volatile __bit DACPSS1 __at(0x8C3);


extern volatile __bit DACR0 __at(0x8C8);


extern volatile __bit DACR1 __at(0x8C9);


extern volatile __bit DACR2 __at(0x8CA);


extern volatile __bit DACR3 __at(0x8CB);


extern volatile __bit DACR4 __at(0x8CC);


extern volatile __bit DC __at(0x19);


extern volatile __bit DC1B0 __at(0x149C);


extern volatile __bit DC1B1 __at(0x149D);


extern volatile __bit DC_SHAD __at(0x7F21);


extern volatile __bit DHEN __at(0x10B8);


extern volatile __bit D_nA __at(0x10A5);


extern volatile __bit EEIE __at(0x494);


extern volatile __bit EEIF __at(0x94);


extern volatile __bit EEPGD __at(0xCAF);


extern volatile __bit FERR __at(0xCEA);


extern volatile __bit FREE __at(0xCAC);


extern volatile __bit FVREN __at(0x8BF);


extern volatile __bit FVRRDY __at(0x8BE);


extern volatile __bit GCEN __at(0x10B7);


extern volatile __bit GIE __at(0x5F);


extern volatile __bit GO __at(0x4E9);


extern volatile __bit GO_nDONE __at(0x4E9);


extern volatile __bit HFIOFL __at(0x4D3);


extern volatile __bit HFIOFR __at(0x4D4);


extern volatile __bit HFIOFS __at(0x4D0);


extern volatile __bit INTE __at(0x5C);


extern volatile __bit INTEDG __at(0x4AE);


extern volatile __bit INTF __at(0x59);


extern volatile __bit IOCAF0 __at(0x1C98);


extern volatile __bit IOCAF1 __at(0x1C99);


extern volatile __bit IOCAF2 __at(0x1C9A);


extern volatile __bit IOCAF3 __at(0x1C9B);


extern volatile __bit IOCAF4 __at(0x1C9C);


extern volatile __bit IOCAF5 __at(0x1C9D);


extern volatile __bit IOCAN0 __at(0x1C90);


extern volatile __bit IOCAN1 __at(0x1C91);


extern volatile __bit IOCAN2 __at(0x1C92);


extern volatile __bit IOCAN3 __at(0x1C93);


extern volatile __bit IOCAN4 __at(0x1C94);


extern volatile __bit IOCAN5 __at(0x1C95);


extern volatile __bit IOCAP0 __at(0x1C88);


extern volatile __bit IOCAP1 __at(0x1C89);


extern volatile __bit IOCAP2 __at(0x1C8A);


extern volatile __bit IOCAP3 __at(0x1C8B);


extern volatile __bit IOCAP4 __at(0x1C8C);


extern volatile __bit IOCAP5 __at(0x1C8D);


extern volatile __bit IOCIE __at(0x5B);


extern volatile __bit IOCIF __at(0x58);


extern volatile __bit IRCF0 __at(0x4CB);


extern volatile __bit IRCF1 __at(0x4CC);


extern volatile __bit IRCF2 __at(0x4CD);


extern volatile __bit IRCF3 __at(0x4CE);


extern volatile __bit LATA0 __at(0x860);


extern volatile __bit LATA1 __at(0x861);


extern volatile __bit LATA2 __at(0x862);


extern volatile __bit LATA4 __at(0x864);


extern volatile __bit LATA5 __at(0x865);


extern volatile __bit LFIOFR __at(0x4D1);


extern volatile __bit LWLO __at(0xCAD);


extern volatile __bit MC1OUT __at(0x8A8);


extern volatile __bit MDBIT __at(0x1CE0);


extern volatile __bit MDCH0 __at(0x1CF8);


extern volatile __bit MDCH1 __at(0x1CF9);


extern volatile __bit MDCH2 __at(0x1CFA);


extern volatile __bit MDCH3 __at(0x1CFB);


extern volatile __bit MDCHODIS __at(0x1CFF);


extern volatile __bit MDCHPOL __at(0x1CFE);


extern volatile __bit MDCHSYNC __at(0x1CFD);


extern volatile __bit MDCL0 __at(0x1CF0);


extern volatile __bit MDCL1 __at(0x1CF1);


extern volatile __bit MDCL2 __at(0x1CF2);


extern volatile __bit MDCL3 __at(0x1CF3);


extern volatile __bit MDCLODIS __at(0x1CF7);


extern volatile __bit MDCLPOL __at(0x1CF6);


extern volatile __bit MDCLSYNC __at(0x1CF5);


extern volatile __bit MDEN __at(0x1CE7);


extern volatile __bit MDMS0 __at(0x1CE8);


extern volatile __bit MDMS1 __at(0x1CE9);


extern volatile __bit MDMS2 __at(0x1CEA);


extern volatile __bit MDMS3 __at(0x1CEB);


extern volatile __bit MDMSODIS __at(0x1CEF);


extern volatile __bit MDOE __at(0x1CE6);


extern volatile __bit MDOPOL __at(0x1CE4);


extern volatile __bit MDOUT __at(0x1CE3);


extern volatile __bit MDSLR __at(0x1CE5);


extern volatile __bit MFIOFR __at(0x4D2);


extern volatile __bit OERR __at(0xCE9);


extern volatile __bit OSFIE __at(0x497);


extern volatile __bit OSFIF __at(0x97);


extern volatile __bit OSTS __at(0x4D5);


extern volatile __bit P1BSEL __at(0x8E9);


extern volatile __bit P1DC0 __at(0x14A0);


extern volatile __bit P1DC1 __at(0x14A1);


extern volatile __bit P1DC2 __at(0x14A2);


extern volatile __bit P1DC3 __at(0x14A3);


extern volatile __bit P1DC4 __at(0x14A4);


extern volatile __bit P1DC5 __at(0x14A5);


extern volatile __bit P1DC6 __at(0x14A6);


extern volatile __bit P1M0 __at(0x149E);


extern volatile __bit P1M1 __at(0x149F);


extern volatile __bit P1RSEN __at(0x14A7);


extern volatile __bit PCIE __at(0x10BE);


extern volatile __bit PEIE __at(0x5E);


extern volatile __bit PEN __at(0x10B2);


extern volatile __bit PLLR __at(0x4D6);


extern volatile __bit PS0 __at(0x4A8);


extern volatile __bit PS1 __at(0x4A9);


extern volatile __bit PS2 __at(0x4AA);


extern volatile __bit PSA __at(0x4AB);


extern volatile __bit PSS1AC0 __at(0x14AA);


extern volatile __bit PSS1AC1 __at(0x14AB);


extern volatile __bit PSS1BD0 __at(0x14A8);


extern volatile __bit PSS1BD1 __at(0x14A9);


extern volatile __bit RA0 __at(0x60);


extern volatile __bit RA1 __at(0x61);


extern volatile __bit RA2 __at(0x62);


extern volatile __bit RA3 __at(0x63);


extern volatile __bit RA4 __at(0x64);


extern volatile __bit RA5 __at(0x65);


extern volatile __bit RCEN __at(0x10B3);


extern volatile __bit RCIDL __at(0xCFE);


extern volatile __bit RCIE __at(0x48D);


extern volatile __bit RCIF __at(0x8D);


extern volatile __bit RD __at(0xCA8);


extern volatile __bit RSEN __at(0x10B1);


extern volatile __bit RX9 __at(0xCEE);


extern volatile __bit RX9D __at(0xCE8);


extern volatile __bit RXDTSEL __at(0x8EF);


extern volatile __bit R_nW __at(0x10A2);


extern volatile __bit SBCDE __at(0x10BA);


extern volatile __bit SBOREN __at(0x8B7);


extern volatile __bit SCIE __at(0x10BD);


extern volatile __bit SCKP __at(0xCFC);


extern volatile __bit SCS0 __at(0x4C8);


extern volatile __bit SCS1 __at(0x4C9);


extern volatile __bit SDAHT __at(0x10BB);


extern volatile __bit SDO1SEL __at(0x8EE);


extern volatile __bit SDOSEL __at(0x8EE);


extern volatile __bit SEN __at(0x10B0);


extern volatile __bit SENDB __at(0xCF3);


extern volatile __bit SMP __at(0x10A7);


extern volatile __bit SPEN __at(0xCEF);


extern volatile __bit SPLLEN __at(0x4CF);


extern volatile __bit SRCLK0 __at(0x8D4);


extern volatile __bit SRCLK1 __at(0x8D5);


extern volatile __bit SRCLK2 __at(0x8D6);


extern volatile __bit SREN __at(0xCED);


extern volatile __bit SRLEN __at(0x8D7);


extern volatile __bit SRNQEN __at(0x8D2);


extern volatile __bit SRPR __at(0x8D0);


extern volatile __bit SRPS __at(0x8D1);


extern volatile __bit SRQEN __at(0x8D3);


extern volatile __bit SRRC1E __at(0x8D8);


extern volatile __bit SRRCKE __at(0x8DA);


extern volatile __bit SRRPE __at(0x8DB);


extern volatile __bit SRSC1E __at(0x8DC);


extern volatile __bit SRSCKE __at(0x8DE);


extern volatile __bit SRSPE __at(0x8DF);


extern volatile __bit SS1SEL __at(0x8ED);


extern volatile __bit SSP1IE __at(0x48B);


extern volatile __bit SSP1IF __at(0x8B);


extern volatile __bit SSPEN __at(0x10AD);


extern volatile __bit SSPM0 __at(0x10A8);


extern volatile __bit SSPM1 __at(0x10A9);


extern volatile __bit SSPM2 __at(0x10AA);


extern volatile __bit SSPM3 __at(0x10AB);


extern volatile __bit SSPOV __at(0x10AE);


extern volatile __bit SSSEL __at(0x8ED);


extern volatile __bit STKOVF __at(0x4B7);


extern volatile __bit STKUNF __at(0x4B6);


extern volatile __bit STR1A __at(0x14B0);


extern volatile __bit STR1B __at(0x14B1);


extern volatile __bit STR1SYNC __at(0x14B4);


extern volatile __bit SWDTEN __at(0x4B8);


extern volatile __bit SYNC __at(0xCF4);


extern volatile __bit T0CS __at(0x4AD);


extern volatile __bit T0IE __at(0x5D);


extern volatile __bit T0IF __at(0x5A);


extern volatile __bit T0SE __at(0x4AC);


extern volatile __bit T0XCS __at(0xF0);


extern volatile __bit T1CKPS0 __at(0xC4);


extern volatile __bit T1CKPS1 __at(0xC5);


extern volatile __bit T1GGO __at(0xCB);


extern volatile __bit T1GGO_nDONE __at(0xCB);


extern volatile __bit T1GPOL __at(0xCE);


extern volatile __bit T1GSEL __at(0x8EB);


extern volatile __bit T1GSPM __at(0xCC);


extern volatile __bit T1GSS0 __at(0xC8);


extern volatile __bit T1GSS1 __at(0xC9);


extern volatile __bit T1GTM __at(0xCD);


extern volatile __bit T1GVAL __at(0xCA);


extern volatile __bit T1OSCEN __at(0xC3);


extern volatile __bit T1OSCR __at(0x4D7);


extern volatile __bit T2CKPS0 __at(0xE0);


extern volatile __bit T2CKPS1 __at(0xE1);


extern volatile __bit T2OUTPS0 __at(0xE3);


extern volatile __bit T2OUTPS1 __at(0xE4);


extern volatile __bit T2OUTPS2 __at(0xE5);


extern volatile __bit T2OUTPS3 __at(0xE6);


extern volatile __bit TMR0CS __at(0x4AD);


extern volatile __bit TMR0IE __at(0x5D);


extern volatile __bit TMR0IF __at(0x5A);


extern volatile __bit TMR0SE __at(0x4AC);


extern volatile __bit TMR1CS0 __at(0xC6);


extern volatile __bit TMR1CS1 __at(0xC7);


extern volatile __bit TMR1GE __at(0xCF);


extern volatile __bit TMR1GIE __at(0x48F);


extern volatile __bit TMR1GIF __at(0x8F);


extern volatile __bit TMR1IE __at(0x488);


extern volatile __bit TMR1IF __at(0x88);


extern volatile __bit TMR1ON __at(0xC0);


extern volatile __bit TMR2IE __at(0x489);


extern volatile __bit TMR2IF __at(0x89);


extern volatile __bit TMR2ON __at(0xE2);


extern volatile __bit TRISA0 __at(0x460);


extern volatile __bit TRISA1 __at(0x461);


extern volatile __bit TRISA2 __at(0x462);


extern volatile __bit TRISA3 __at(0x463);


extern volatile __bit TRISA4 __at(0x464);


extern volatile __bit TRISA5 __at(0x465);


extern volatile __bit TRMT __at(0xCF1);


extern volatile __bit TSEN __at(0x8BD);


extern volatile __bit TSRNG __at(0x8BC);


extern volatile __bit TUN0 __at(0x4C0);


extern volatile __bit TUN1 __at(0x4C1);


extern volatile __bit TUN2 __at(0x4C2);


extern volatile __bit TUN3 __at(0x4C3);


extern volatile __bit TUN4 __at(0x4C4);


extern volatile __bit TUN5 __at(0x4C5);


extern volatile __bit TX9 __at(0xCF6);


extern volatile __bit TX9D __at(0xCF0);


extern volatile __bit TXCKSEL __at(0x8EA);


extern volatile __bit TXEN __at(0xCF5);


extern volatile __bit TXIE __at(0x48C);


extern volatile __bit TXIF __at(0x8C);


extern volatile __bit UA __at(0x10A1);


extern volatile __bit VREGPM0 __at(0xCB8);


extern volatile __bit VREGPM1 __at(0xCB9);


extern volatile __bit WCOL __at(0x10AF);


extern volatile __bit WDTPS0 __at(0x4B9);


extern volatile __bit WDTPS1 __at(0x4BA);


extern volatile __bit WDTPS2 __at(0x4BB);


extern volatile __bit WDTPS3 __at(0x4BC);


extern volatile __bit WDTPS4 __at(0x4BD);


extern volatile __bit WPUA0 __at(0x1060);


extern volatile __bit WPUA1 __at(0x1061);


extern volatile __bit WPUA2 __at(0x1062);


extern volatile __bit WPUA3 __at(0x1063);


extern volatile __bit WPUA4 __at(0x1064);


extern volatile __bit WPUA5 __at(0x1065);


extern volatile __bit WR __at(0xCA9);


extern volatile __bit WREN __at(0xCAA);


extern volatile __bit WRERR __at(0xCAB);


extern volatile __bit WUE __at(0xCF9);


extern volatile __bit ZERO __at(0x1A);


extern volatile __bit Z_SHAD __at(0x7F22);


extern volatile __bit nBOR __at(0x4B0);


extern volatile __bit nPD __at(0x1B);


extern volatile __bit nPOR __at(0x4B1);


extern volatile __bit nRI __at(0x4B2);


extern volatile __bit nRMCLR __at(0x4B3);


extern volatile __bit nT1SYNC __at(0xC2);


extern volatile __bit nTO __at(0x1C);


extern volatile __bit nWPUEN __at(0x4AF);


# 30 "C:\Program Files (x86)\Microchip\xc8\v2.00\pic\include\pic.h"
#pragma intrinsic(__nop)
extern void __nop(void);

# 78
__attribute__((__unsupported__("The " "FLASH_READ" " macro function is no longer supported. Please use the MPLAB X MCC."))) unsigned char __flash_read(unsigned short addr);

__attribute__((__unsupported__("The " "FLASH_WRITE" " macro function is no longer supported. Please use the MPLAB X MCC."))) void __flash_write(unsigned short addr, unsigned short data);

__attribute__((__unsupported__("The " "FLASH_ERASE" " macro function is no longer supported. Please use the MPLAB X MCC."))) void __flash_erase(unsigned short addr);

# 114 "C:\Program Files (x86)\Microchip\xc8\v2.00\pic\include\eeprom_routines.h"
extern void eeprom_write(unsigned char addr, unsigned char value);
extern unsigned char eeprom_read(unsigned char addr);


# 91 "C:\Program Files (x86)\Microchip\xc8\v2.00\pic\include\pic.h"
#pragma intrinsic(_delay)
extern __nonreentrant void _delay(unsigned long);
#pragma intrinsic(_delaywdt)
extern __nonreentrant void _delaywdt(unsigned long);

#pragma intrinsic(_delay3)
extern __nonreentrant void _delay3(unsigned char);

# 137
extern __bank0 unsigned char __resetbits;
extern __bank0 __bit __powerdown;
extern __bank0 __bit __timeout;

# 41 "main.c"
volatile union _I2C_buffer
{
struct _data
{
unsigned char ID;
unsigned char ADDRESS;
unsigned char START_STOP;
unsigned char IOWPU;
unsigned char MODE;
unsigned char SAVE;
unsigned char RESET;
unsigned int GEAR_RATIO;
unsigned int DIAMETER;
int RPM;
int SPEED;
long DISTANCE;
float RPM_PID_KP;
float RPM_PID_KD;
float RPM_PID_KI;
float ATS_PID_KP;
float ATS_PID_KD;
float ATS_PID_KI;
} data;
unsigned char byte[];
} I2C_buffer;

const unsigned char RX_ELMNTS = sizeof (I2C_buffer);
unsigned char first_i2c = 1;
unsigned char index_i2c = 0;
unsigned short counter = 0;
int lset = 0;
bit zero_cross = 0;
float accumulator = 0;
float lasterror = 0;



void __interrupt isr() {

if (INTCONbits.IOCIF == 1 && IOCAFbits.IOCAF4 == 1)
{
INTCONbits.IOCIE = 0;
counter++;
if (LATAbits.LATA0 == 0) {
I2C_buffer.data.DISTANCE--;
}
else
{
I2C_buffer.data.DISTANCE++;
}
IOCAFbits.IOCAF4 = 0;
INTCONbits.IOCIE = 1;
}

if (PIR1bits.TMR1IF == 1)
{
INTCONbits.IOCIE = 0;
T1CONbits.TMR1ON = 0;
I2C_buffer.data.RPM = (counter * 300) / I2C_buffer.data.GEAR_RATIO;
counter = 0;
if (LATAbits.LATA0 == 0) {
I2C_buffer.data.RPM = I2C_buffer.data.RPM *-1;
}
INTCONbits.IOCIE = 1;
PIR1bits.TMR1IF = 0;
T1CONbits.TMR1ON = 1;
}

static unsigned char junk = 0;

if (PIR1bits.SSP1IF)
{
INTCONbits.IOCIE = 0;
PIE1bits.TMR1IE = 0;
if (SSP1STATbits.R_nW)
{
if (!SSP1STATbits.D_nA)
{
SSP1BUF = I2C_buffer.byte[index_i2c++];
SSP1CON1bits.CKP = 1;
}
if (SSP1STATbits.D_nA)
{
SSP1BUF = I2C_buffer.byte[index_i2c++];
SSP1CON1bits.CKP = 1;
}
}
if (!SSP1STATbits.R_nW)
{
if (!SSP1STATbits.D_nA)
{
first_i2c = 1;
junk = SSP1BUF;
SSP1CON1bits.CKP = 1;
}
if (SSP1STATbits.D_nA)
{
if (first_i2c) {
index_i2c = SSP1BUF;
first_i2c = 0;
}

else {
if (index_i2c < RX_ELMNTS)
{
I2C_buffer.byte[index_i2c++] = SSP1BUF;
}
else {
junk = SSP1BUF;
}
}
if (SSP1CON1bits.WCOL)
{
SSP1CON1bits.WCOL = 0;
junk = SSP1BUF;
}
SSP1CON1bits.CKP = 1;
}
}
PIR1bits.SSP1IF = 0;
INTCONbits.IOCIE = 1;
PIE1bits.TMR1IE = 1;
}

if (PIR2bits.BCL1IF)
{
junk = SSP1BUF;
PIR2bits.BCL1IF = 0;
SSP1CON1bits.CKP = 1;
PIR1bits.SSP1IF = 0;
}
}



PWM_Init(void)
{

# 197
PR2 = 0xFF;
CCP1CON = 0b00001100;
CCPR1L = 0b00000000;
PIR1bits.TMR2IF = 0;
T2CON = 0b00000100;

# 211
}

PWM_set_duty(int duty)
{
if (duty < 1024) {
CCPR1L = (0xFF & ((unsigned int) duty >> 2));
CCP1CON = (0x0C | (0x30 & ((unsigned int) duty << 4)));

}
}

void M_control(int ctr)
{
if (ctr == 0)
{
PWM_set_duty(ctr);
}
if (ctr > 0)
{
LATAbits.LATA0 = 1;
PWM_set_duty(ctr);
}
if (ctr < 0)
{
LATAbits.LATA0 = 0;
ctr = ctr * -1;
PWM_set_duty(ctr);
}
}

void calculate_pid(int set)
{
float error = 0;
float pid = 0;
error = set - I2C_buffer.data.RPM;
pid = error * I2C_buffer.data.RPM_PID_KP;
accumulator += error;
pid += I2C_buffer.data.RPM_PID_KI*accumulator;
pid += I2C_buffer.data.RPM_PID_KD * (error - lasterror);
lasterror = error;
if (pid >= 1023)
{
pid = 1023;
}
if (pid <= -1023)
{
pid = -1023;
}
M_control((int) pid);
}

void PID(int set)
{
zero_cross = ((lset^set) < 0);
if (((zero_cross == 1) &(set <= 0) & (lset <= 0)))
{
zero_cross = 0;
}
if (zero_cross == 1) {
do {
calculate_pid(0);
_delay((unsigned long)((10)*(32000000/4000.0)));
} while ((I2C_buffer.data.RPM != 0));
}
lset = set;
calculate_pid(set);
}

# 286
void init_I2C_buffer() {
I2C_buffer.data.ID = 0xF3;
I2C_buffer.data.ADDRESS = 0x24 << 1;
I2C_buffer.data.START_STOP = 0;
I2C_buffer.data.IOWPU = 1;
I2C_buffer.data.MODE = 5;
I2C_buffer.data.SAVE = 0;
I2C_buffer.data.RESET = 0;
I2C_buffer.data.GEAR_RATIO = 150;
I2C_buffer.data.DIAMETER = 42;
I2C_buffer.data.RPM = 0;
I2C_buffer.data.SPEED = 0;
I2C_buffer.data.DISTANCE = 0;
I2C_buffer.data.RPM_PID_KP = 0.1;
I2C_buffer.data.RPM_PID_KD = 0.01;
I2C_buffer.data.RPM_PID_KI = 0.04;
I2C_buffer.data.ATS_PID_KP = 0.15;
I2C_buffer.data.ATS_PID_KD = 0.08;
I2C_buffer.data.ATS_PID_KI = 0.03;
}



void main() {
OSCCON = 0b11110000;
init_I2C_buffer();
TRISA = 0b00011110;
ANSELA = 0b00000000;
WPUA = 0b00011110;
OPTION_REGbits.nWPUEN = 0;
APFCONbits.CCP1SEL = 1;
LATAbits.LATA0 = 0;
SSP1STAT = 0b10000000;
SSP1CON1 = 0b00110110;
SSP1CON2bits.SEN = 1;
SSP1CON3bits.BOEN = 1;
SSP1CON3bits.SDAHT = 1;
SSP1CON3bits.SBCDE = 1;
SSP1ADD = I2C_buffer.data.ADDRESS;
PIR1bits.SSP1IF = 0;
PIR2bits.BCL1IF = 0;
PIE2bits.BCL1IE = 1;
PIE1bits.SSP1IE = 1;
INTCONbits.PEIE = 1;
PWM_Init();
PWM_set_duty(0);
IOCANbits.IOCAN4 = 1;
INTCON = 0b01001000;
T1CON = 0b00110100;
PIE1bits.TMR1IE = 1;
T1CONbits.TMR1ON = 1;
INTCONbits.GIE = 1;

while (1) {
PID(I2C_buffer.data.SPEED);
_delay((unsigned long)((10)*(32000000/4000.0)));
}

# 355
}

