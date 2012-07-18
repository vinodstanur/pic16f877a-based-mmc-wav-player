/*
------------------------------------------------------------
PIC16F877A + MMC WAV PLAYER
(code updated on 06/09/2011)
improved audio quality compared to
previous version.
------------------------------------------------------------
Compiler -High Tech C
------------------------------------------------------------
Author: Vinod S
<vinodstanur@gmail.com>
http://vinodstanur.blogspot.com
------------------------------------------------------------
*/
#include<pic.h>
#define _XTAL_FREQ 20e6
#define CS RC2
#define RS RB2
#define EN RB1
#define fst cmd(0x80)
#define snd cmd(0xc0)
unsigned int i, g, trickl, trickr;
unsigned long int lg;
unsigned int fat_start, dir_start, count, data_start, scan, numm, numm1;
unsigned char buf[64], play1[64], play2[64], no, status, sect_per_clst, bb;
unsigned long int long1, long2, bitrate;
unsigned char readdata, readdata2, u, speed, check, zz, zx;
unsigned int count;
float flt;
bit toggle;
bit set;
/*-----------------LCD BEGIN----------------------------*/
void LCD_STROBE(void)
{
    EN = 1;
    __delay_us(0.5);
    EN = 0;
}
 
void data(unsigned char c)
{
    RS = 1;
    __delay_us(40);
    PORTD = (c >> 4);
    LCD_STROBE();
    PORTD = (c);
    LCD_STROBE();
}
 
void cmd(unsigned char c)
{
    RS = 0;
    __delay_us(40);
    PORTD = (c >> 4);
    LCD_STROBE();
    PORTD = (c);
    LCD_STROBE();
}
 
void clear(void)
{
    cmd(0x01);
    __delay_ms(2);
}
 
void lcd_init()
{
    __delay_ms(20);
    cmd(0x30);
    __delay_ms(1);
    cmd(0x30);
    __delay_ms(1);
    cmd(0x30);
    cmd(0x28);            // Function set (4-bit interface, 2 lines, 5*7Pixels)
    cmd(0x28);            // Function set (4-bit interface, 2 lines, 5*7Pixels)
    cmd(0x28);            // Function set (4-bit interface, 2 lines, 5*7Pixels)
    cmd(0x0c);            // Make cursorinvisible
    clear();
    clear();            // Clear screen
    cmd(0x6);            // Set entry Mode
}
 
void string(const char *q)
{
    while (*q) {
        data(*q++);
    }
}
 
void istring(unsigned int q)
{
    data(48 + (q / 10000));
    q %= 10000;
    data(48 + (q / 1000));
    q %= 1000;
    data(48 + (q / 100));
    q %= 100;
    data(48 + (q / 10));
    q %= 10;
    data(48 + (q));
}
 
void itostr(unsigned long int ii)
{
	cmd(0x81);
	char uu = 0;
	unsigned char lcd_bufff[16];
	while(ii) {
		lcd_bufff[uu++] = (ii % 10 + '0');
		ii /= 10;
	}
	while(uu) data(lcd_bufff[--uu]);
}

/*
itostr(unsigned long int q)
{
    cmd(0x81);
    data(48 + (q / 1000000000));
    q %= 1000000000;
    data(48 + (q / 100000000));
    q %= 100000000;
    data(48 + (q / 10000000));
    q %= 10000000;
    data(48 + (q / 1000000));
    q %= 1000000;
    data(48 + (q / 100000));
    q %= 100000;
    data(48 + (q / 10000));
    q %= 10000;
    data(48 + (q / 1000));
    q %= 1000;
    data(48 + (q / 100));
    q %= 100;
    data(48 + (q / 10));
    q %= 10;
    data(48 + (q));
}
*/
void cstring(unsigned int q)
{
    data(48 + (q / 100));
    q %= 100;
    data(48 + (q / 10));
    q %= 10;
    data(48 + (q));
}
 
/*-----------------------LCD END--------------------*/
/*-----------------------USRT BEGIN--------------------*/
void usrt_init()
{
    TRISC6 = 0;
    TXSTA = 0b00100110;
    RCSTA = 0b11010000;
    SPBRG = 10;
}
 
void printf(const char *p)
{
    while (*p) {
        TXREG = *p;
        while (TRMT == 0);
        p++;
    }
}
 
void txd(unsigned char vv)
{
    TXREG = vv;
    while (TRMT == 0);
}
 
/*-----------------------USRT END---------------------*/
 
/*----------------------PWM BEGINS--------------------*/
void pwm_init()
{
    TRISC1 = 0;
    T2CKPS1 = 0;
    T2CKPS0 = 0;
    PR2 = 200;
    CCPR2L = 0x10;
    TMR2ON = 1;
    CCP2CON = 0b00001100;
}
 
void pwm_disable()
{
    CCP2CON = 0b00000000;
}
 
void pwm_enable()
{
    CCP2CON = 0b00001100;
}
 
/*--------------------PWM END------------------------*/
/*----------------------TIMER1 BEGINS--------------------*/
void timer1_init()
{
    T1CON = 0b00000001;
    TMR1IE = 1;
    GIE = 1;
    PEIE = 1;
}
 
void timer0_init()
{
    OPTION_REG = 0b00000000;
    T0IE = 1;
    GIE = 1;
}
 
/*--------------------TIMER1 END------------------------*/
/*-------------------MMC BEGIN-----------------------*/
void spi_init()
{
    TRISC4 = 1;
    RC2 = 1;
    RC3 = 0;
    RC5 = 0;
    TRISC2 = TRISC3 = TRISC5 = 0;
    SSPCON = 0b00100010;
    SSPEN = 1;
    SMP = 0;
    CKE = 1;
    CKP = 0;
}
 
void spi_write(unsigned char kk)
{
    SSPBUF = kk;
    while (BF == 0);
}
 
void spi_read()
{
    SSPBUF = 0xff;
    while (BF == 0);
    readdata = SSPBUF;
}
 
void command(char command, unsigned long int fourbyte_arg, char CRCbits)
{
    spi_write(0xff);
    spi_write(0b01000000 | command);
    spi_write((unsigned char) (fourbyte_arg >> 24));
    spi_write((unsigned char) (fourbyte_arg >> 16));
    spi_write((unsigned char) (fourbyte_arg >> 8));
    spi_write((unsigned char) fourbyte_arg);
    spi_write(CRCbits);
    spi_read();
}
 
void mmc_init()
{
    CS = 1;
    for (u = 0; u < 50; u++) {
        spi_write(0xff);
    }
    CS = 0;
    __delay_ms(1);
    command(0, 0, 0x95);
    count = 0;
    while ((readdata != 1) && (count < 1000)) {
        spi_read();
        count++;
    }
    if (count >= 1000) {
        string("CARD ERROR-CMD0 ");
        while (1);
    }
    command(1, 0, 0xff);
    count = 0;
    while ((readdata != 0) && (count < 1000)) {
        command(1, 0, 0xff);
        spi_read();
        count++;
    }
    if (count >= 1000) {
        string("CARD ERROR-CMD1 ");
        while (1);
    }
    command(16, 512, 0xff);
    count = 0;
    while ((readdata != 0) && (count < 1000)) {
        spi_read();
        count++;
    }
    if (count >= 1000) {
        string("CARD ERROR-CMD16");
        while (1);
    }
    string("MMC INITIALIZED!");
    __delay_ms(290);
    SSPCON = SSPCON & 0b11111101;
}
 
void write(unsigned char dataa, unsigned long int sector)
{
    long1 = sector;
    long1 *= 512;
    command(24, long1, 0xff);
    while (readdata != 0) {
        spi_read();
    }
    spi_write(0xff);
    spi_write(0xff);
    spi_write(0b11111110);
    for (int g = 0; g < 512; g++) {
        spi_write(dataa);
    }
    spi_write(0xff);
    spi_write(0xff);
    spi_read();
    while ((readdata & 0b00011111) != 0x05) {
        spi_read();
    }
    while (readdata != 0xff) {
        spi_read();
    }
}
 
void load_to_buf(unsigned long int sector, unsigned char num)
{
    long1 = sector;
    long1 *= 512;
    command(17, long1, 0xff);
    while (readdata != 0) {
        spi_read();
    }
    while (readdata != 0xfe) {
        spi_read();
    }
    trickl = 64;
    trickl *= num;
    trickr = 448 - trickl;
    for (g = 0; g < trickl; g++) {
        spi_read();
    }
    for (g = 0; g < 64; g++) {
        spi_read();
        buf[g] = readdata;
    }
    for (g = 0; g < trickr; g++) {
        spi_read();
    }
    spi_write(0xff);
    spi_write(0xff);
}
 
void read(unsigned long int c)
{
    for (i = 0; i <= sect_per_clst; i++) {
        long1 = 512 * c;
        command(17, long1, 0xff);
        while (readdata != 0) {
            spi_read();
        }
        while (readdata != 0xfe) {
            spi_read();
        }
        for (g = 0; g < 4; g++) {
            while (toggle == 0);
            {
                for (zx = 0; zx < 64; zx++) {
                    spi_read();
                    play2[zx] = readdata;
                }
            }
            while (toggle == 1);
            {
                for (zx = 0; zx < 64; zx++) {
                    spi_read();
                    play1[zx] = readdata;
                }
            }
        }
        spi_write(0xff);
        spi_write(0xff);
        c += 1;
    }
}
 
/*--------------------------FAT ACCESS-----------------------------*/
void fat_init()            //BOOT SECTOR SCANNING//
{
    load_to_buf(0, 0);
    fat_start = buf[0x0e];
    dir_start = (fat_start + (((buf[0x17] << 8) + buf[0x16]) * 2));
    data_start = (dir_start + ((((buf[0x12] << 8) + (buf[0x11])) * 32) / 512));
    sect_per_clst = buf[0x0d];
}
 
unsigned int find_next_cluster(unsigned int clstt)
{
    numm1 = (2 * (clstt % 256));
    load_to_buf((fat_start + (clstt / 256)), numm1 / 64);
    numm = numm1 % 64;
    return ((buf[numm + 1] << 8) + buf[numm]);
}
 
void play_file(unsigned int clstr)
{
    long2 = clstr - 2;
    long2 *= sect_per_clst;
    long2 += data_start;
    load_to_buf(long2, 0);
    for (u = 31; u > 27; u--) {
        bitrate <<= 8;
        bitrate += buf[u];
    }
    itostr(bitrate);
    flt = 1.4 * 1250000 / bitrate;
    bb = 255 - flt;
    cmd(0xc0);
    istring(bb);
    TMR0IE = 1;
    while (clstr != 0xffff) {
        long2 = clstr - 2;
        long2 *= sect_per_clst;
        long2 += data_start;
        read(long2);
        if (((buf[numm + 3] << 8) + buf[numm + 2]) != (clstr + 1)) {
            clstr = find_next_cluster(clstr);
            } else {
            clstr++;
            numm++;
        }
    }
    cmd(0xc0);
    string("read complete   ");
    __delay_ms(1000);
    TMR0IE = 0;
}
 
void file_scan()
{
    scan:scan = dir_start;
    no = 0;
    while (1) {
        load_to_buf(scan, no);
        if ((buf[1] != 0) && (buf[2] != 0) && (buf[0] != 0xe5) && (buf[0] != 0x00) && ((buf[11]) & 0b00010000) == 0) {
            fst;
            clear();
            for (g = 0; g < 11; g++) {
                data(buf[g]);
            }
            for (lg = 0; lg < 140000; lg++) {
                if (RE0 == 1) {
                    status = 1;
                    break;
                }
            }
        }
        if (status == 1) {
            play_file((buf[27] << 8) + buf[26]);
            status = 0;
            break;
        }
        if (buf[32] == 0) {
            break;
        }
        if ((buf[33] != 0) && (buf[34] != 0) && (buf[32] != 0xe5) && ((buf[43]) & 0b00010000) == 0) {
            fst;
            clear();
            for (g = 32; g < 43; g++) {
                data(buf[g]);
            }
            for (lg = 0; lg < 140000; lg++) {
                if (RE0 == 1) {
                    status = 2;
                    break;
                }
            }
            fst;
            clear();
        }
        no++;
        if (no == 8) {
            no = 1;
            scan += 1;
        }
        if (status == 2) {
            play_file((buf[27 + 32] << 8) + buf[26 + 32]);
            status = 0;
            break;
        }
    }
    goto scan;
}
 
/*------------------------------mmc end-------------------------------*/
/*---------------------------ADC functions----------------------------*/
void adc_init()
{
    TRISA0 = 1;
    ADCON0 = 0b10000001;
    ADCON1 = 0b10001110;
}
 
/*---------------------------ADC END----------------------------*/
void interrupt timer()
{
    if (toggle == 1) {
        CCPR2L = play1[zz];
        zz++;
        if (zz == 64) {
            zz = 0;
            toggle = 0;
        }
        } else if (toggle == 0) {
        CCPR2L = play2[zz];
        zz++;
        if (zz == 64) {
            zz = 0;
            toggle = 1;
        }
    }
    TMR0 = bb;
    TMR0IF = 0;
}
 
main()
{
    CS = 1;
    PORTD = 0;
    TRISC4 = 0;
    TRISC5 = 0;
    TRISD = 0;
    TRISB2 = 0;
    TRISB1 = 0;
    TRISB6 = 0;
    TRISB7 = 0;
    TRISE0 = 1;
    TRISE1 = 1;
    lcd_init();
    adc_init();
    usrt_init();
    spi_init();
    mmc_init();
    fat_init();
    timer0_init();
    pwm_init();
    pwm_enable();
    speed = 0;
    RB7 = 0;
    count = 0;
    CS = 0;
    no = 0;
    zz = 0;
    while (1) {
        file_scan();
        for (;;);
    }
}
