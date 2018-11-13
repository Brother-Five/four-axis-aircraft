#ifndef __ADD_BULLET_H
#define __ADD_BULLET_H
#ifdef __cplusplus
#include "bsp.h"
extern u8 timemark,bumark,bu_mark,vmark,fumark,lmark,rmark,cir_mark;
extern int16_t rx,rrx,ry,rry,rth,rrth;

void add_base_move(u8 _value);
void add_base_bullet(u8 _value);
u8 Get_bullet_flag(void);
void Set_bullet_flag(u8 tflag);
void add_bullet_run();
void Enter_bullet_mode(u8 _mode);
void add_bullet(CanRxMsg *msg_rece);

void add_bullet1(u8 _value);
void send_cmd();
void rece_usart(u8*);
void go_back();
void send_bit_1(void);
void send_bit_0(void);
void send_start_bit(void);
void send_byte(u16 byte);
//void LED_Init(void);
#endif
#endif
