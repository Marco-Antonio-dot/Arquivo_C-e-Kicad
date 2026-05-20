/*  TG -> Sensor óptico eletrônico para monitorar a qualidade do filtro de ar do motor <-
    Author: Marco Antonio, Danilo,
    Renato, Rodrigo, Raphael.
    
    Crystal 4Mhz.
    Ciclo de maquina 1us.
    Estouro do timer 1, aprx: 65ms.
    Modo Sleep acionado após 4 minutos aproximadamente de funcionamento.
    Data de inicio 15/09/2025.
    Última atualização 20/05/2026.

*/

#define val_limpo   900
#define val_sujo     40
#define N_mediamovel 10


sbit LCD_RS at RB2_bit;
sbit LCD_EN at RB3_bit;
sbit LCD_D4 at RB4_bit;
sbit LCD_D5 at RB5_bit;
sbit LCD_D6 at RB6_bit;
sbit LCD_D7 at RB7_bit;

sbit LCD_RS_Direction at TRISB2_bit;
sbit LCD_EN_Direction at TRISB3_bit;
sbit LCD_D4_Direction at TRISB4_bit;
sbit LCD_D5_Direction at TRISB5_bit;
sbit LCD_D6_Direction at TRISB6_bit;
sbit LCD_D7_Direction at TRISB7_bit;

int ldr, ldr2, aux;
char cen, dez, uni;
char chr, count, vet[7]={0}, txt[7]={0};
char flags, i, j;
bit clr;
unsigned long contador;
short last_read, s1;
char txt1[7]={0}, txt2[7]={0};

float historico_ldr2[N_mediamovel]={0};
int indice_ldr2=0;

void Show_disp();
void Menu1();
void Menu2();
void Clear();
void Butt();
void GravaEEprom();
void ldr1_func();
void Ldr2_func();


void interrupt()
{
    if(tmr1if_bit)
    {
       tmr1if_bit=0x00;
       tmr1h=0x00;
       tmr1l=0x00;
       
       contador++;
       if(contador==4000)
        {
           contador=0x00;
           portc &=~(1<<1);
           
          asm sleep;

        }//end contador
        
    }//end

}//end interrupção

void main()
{
 cmcon=0x07;
 option_reg=0x87;
 intcon=0xc0;
 t1con=0x01;
 tmr1ie_bit=0x01;
 
 adcon0=0x01;
 adcon1=0x40;

 trisa=0x03;
 porta=0x00;

 trisb=0x03;
 portb=0x00;

 trisc=0x90;
 portc=0x2b;
 delay_ms(3000);
 portc=0x01;
 
 portc |=(1<<1);

 t2con=0x07;
 ccp1con=0x0e;
 pr2=0xff;
 ccpr1l=0x00;
 
 delay_ms(1000);
 
 for(i=0; i< N_mediamovel; i++)
  {
     Ldr1_func();
     Ldr2_func();
     delay_ms(100);

  }//end for
  //count=0x01;
 
 lcd_init();
 lcd_cmd(_LCD_CURSOR_OFF);
 lcd_out(1,1,"Iniciando...");
 delay_ms(2000);
 lcd_cmd(_LCD_CLEAR);
 delay_ms(500);
 

 s1=EEPROM_Read(0x0a);
 delay_ms(1);
 bytetostr(s1, txt1);
 lcd_out(1,1,"sensorLed->");
 lcd_out(1, 13, txt1);
 lcd_out_cp("%");
 delay_ms(2000);
 lcd_cmd(_LCD_CLEAR);
 delay_ms(500);

 
last_read = EEPROM_Read(0x00);
delay_ms(1);
bytetostr(last_read, txt2);

lcd_cmd(_LCD_CLEAR);
delay_ms(10);
lcd_out(1, 1, "Ultima leitura salva");
lcd_out(2, 7, txt2);
lcd_chr_cp('%');

delay_ms(1000);

for(i = 0; i < 16; i++)
{
    lcd_cmd(_LCD_SHIFT_LEFT);
    delay_ms(250);
}

for(i = 0; i < 16; i++)
{
    lcd_cmd(_LCD_SHIFT_RIGHT);
    delay_ms(250);
}

delay_ms(1000);
lcd_cmd(_LCD_CLEAR);
lcd_out(1, 1, "Pressione B1.");
delay_ms(500);


  while(1)
  {
     Butt();
     Ldr1_func();
     Ldr2_func();
     GravaEEprom();
     
      switch(count)
        {
          case 1: Show_disp(); break;
          case 2: Menu1();     break;
          case 3: Menu2();     break;

        }//end switch
     
     if(ldr2<20)
      {
        lcd_cmd(_LCD_CLEAR);
        delay_ms(100);
        portc |=(1<<3);
        portc &=~(1<<0);
        
        lcd_chr(1,1,'T');lcd_chr_cp('r');lcd_chr_cp('o'); lcd_chr_cp('c');lcd_chr_cp('a');lcd_chr_cp('r');
        lcd_chr_cp(' ');lcd_chr_cp('F');lcd_chr_cp('i'); lcd_chr_cp('l');lcd_chr_cp('T');lcd_chr_cp('r');
        lcd_chr_cp('o');
        
        for(i=0; i<16; i++)
         {
          lcd_cmd(_LCD_SHIFT_RIGHT);
          delay_ms(100);
          
         }

         for(i=16; i>0; i--)
         {
          lcd_cmd(_LCD_SHIFT_LEFT);
          delay_ms(100);
         }

      }//

      else
      {
        portc &=~(1<<3);
        portc |=(1<<0);
        
        switch(count)
        {
          case 1: Show_disp(); break;
          case 2: Menu1();     break;
          case 3: Menu2();     break;

        }//end switch

      }//end else
     
     while(ldr<30)
      {
         lcd_cmd(_LCD_CLEAR);
         lcd_out(1, 1, "Limpar os Leds");
         lcd_out(2, 5, txt1);
         lcd_out_cp("%");
         
        for(i=0; i<15; i++)
         {
           lcd_cmd(_LCD_SHIFT_RIGHT);
           portc ^=(1<<5);
           delay_ms(100);
           
         }//end for

        for(i=20; i>0; i--)
         {
           lcd_cmd(_LCD_SHIFT_LEFT);
           portc ^=(1<<5);
           delay_ms(100);

         }//end for
         
      }//while finito
      
     delay_ms(100);

  }//end while

}//end main

void Ldr1_func()
{
   ccpr1l=adc_read(0)>>0x02;
   ldr=ccpr1l;
   ldr/=2.55;         //variavel para adequar a conversão ad para exibir
   bytetostr(ldr, txt);    //no display

}//end duty

void Ldr2_func()
{
  /* ldr2=adc_read(1)>>0x02;  //faz a conversaão de 10bits(1023) para 8bits(256)
   ldr2/=2.55;
   inttostr(ldr2, vet);
   delay_ms(100); */
   
   int adc_puro;
   float percentual_calculado;
   float soma = 0;
   int k;

   // 1. Leitura direta do canal 1 (sem deslocamento de bits para manter a precisão)
   adc_puro = adc_read(1);
   
   if(adc_puro>= val_limpo)
     percentual_calculado=100.0;
   else if(adc_puro<= val_sujo)
     percentual_calculado=0.0;
   else
     percentual_calculado = ((float)(adc_puro - val_sujo) / (float)(val_limpo - val_sujo)) * 100.0;
     
   historico_ldr2[indice_ldr2] = percentual_calculado;
   indice_ldr2++;

   if (indice_ldr2 >= N_mediamovel)
   {
       indice_ldr2 = 0;
   }
   
   for(k=0; k< N_mediamovel; k++)
       soma += historico_ldr2[k];
       
   ldr2=(int)(soma/N_mediamovel);
   //inttostr(ldr2, vet);
   
   cen=ldr2/100;
   dez=(ldr2%100)/10;
   uni=ldr2%10;
   
   delay_ms(50);

}//

void Show_disp()
{
  Clear();
  lcd_chr(1,1,'L');
  lcd_chr_cp('e');
  lcd_chr_cp('i');
  lcd_chr_cp('t');
  lcd_chr_cp('u');
  lcd_chr_cp('r');
  lcd_chr_cp('a');
  lcd_chr_cp(' ');
  lcd_chr_cp('d');
  lcd_chr_cp('o');
  lcd_chr_cp(' ');
  lcd_chr_cp('L');
  lcd_chr_cp('e');
  lcd_chr_cp('d');
  lcd_out(2, 7, txt);
  lcd_chr_cp('%');

}//end disp

void Menu1()
{
  Clear();
  lcd_chr(1,1,'Q');
  lcd_chr_cp('u');
  lcd_chr_cp('a');
  lcd_chr_cp('l');
  lcd_chr_cp('i');
  lcd_chr_cp('d');
  lcd_chr_cp('a');
  lcd_chr_cp('d');
  lcd_chr_cp('e');
  lcd_chr_cp('/');
  lcd_chr_cp('F');
  lcd_chr_cp('i');
  lcd_chr_cp('l');
  lcd_chr_cp('t');
  lcd_chr_cp('r');
  lcd_chr_cp('o');
  lcd_chr_cp('2');
  //lcd_out(2, 3, vet);
  lcd_chr(2,6, cen + 0x30);
  lcd_chr_cp(dez + 0x30);
  lcd_chr_cp(uni + 0x30);
  lcd_chr(2,9,'%');

}//end menu

void Menu2()
{
   Clear();
   lcd_chr(1,2,'D');

}//end

void Butt()
{
  if(rb1_bit) flags.b1=0x01;
  if(!rb1_bit && flags.b1)
  {
    flags.b1=0x00;
    count++;
    //count&=3;
    if(count>2) count=0x01;
    clr=0x01;

  }//end flag b1

}//end butt

void Clear()
{
  if(clr)
  {
     clr=0x00;
     lcd_cmd(_LCD_CLEAR);

  }//end if

}//end clear

void GravaEEprom()
{

    if(abs(ldr2-last_read) >10)
     {
        EEPROM_Write(0x00, ldr2);
        delay_ms(20);
        last_read=ldr2;
        bytetostr(last_read, txt2);

     }
        
/*-----------------------------------------------*/
        
        EEPROM_Write(0x0a, ldr);
        delay_ms(30);
        s1=ldr;
        bytetostr(s1, txt1);


}//end