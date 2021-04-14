#ifndef EP_MACRO_H_
  #define EP_MACRO_H_
// some usefull macros for EP handling 
#define ACK_EP1_IN()    UEP1_CTRL &= ~MASK_UEP_T_RES
#define ACK_EP2_IN()    UEP2_CTRL &= ~MASK_UEP_T_RES
#define ACK_EP3_IN()    UEP3_CTRL &= ~MASK_UEP_T_RES
#define ACK_EP4_IN()    UEP4_CTRL &= ~MASK_UEP_T_RES
                      
#define NAK_EP1_IN()    UEP1_CTRL = (UEP1_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_NAK
#define NAK_EP2_IN()    UEP2_CTRL = (UEP2_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_NAK
#define NAK_EP3_IN()    UEP3_CTRL = (UEP3_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_NAK
#define NAK_EP4_IN()    UEP4_CTRL = (UEP4_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_NAK

#define STALL_EP1_IN()  UEP1_CTRL |= UEP_T_RES_STALL
#define STALL_EP2_IN()  UEP2_CTRL |= UEP_T_RES_STALL
#define STALL_EP3_IN()  UEP3_CTRL |= UEP_T_RES_STALL
#define STALL_EP4_IN()  UEP4_CTRL |= UEP_T_RES_STALL

#define ACK_EP1_OUT()   UEP1_CTRL &= ~MASK_UEP_R_RES
#define ACK_EP2_OUT()   UEP2_CTRL &= ~MASK_UEP_R_RES
#define ACK_EP3_OUT()   UEP3_CTRL &= ~MASK_UEP_R_RES
#define ACK_EP4_OUT()   UEP4_CTRL &= ~MASK_UEP_R_RES
                     
#define NAK_EP1_OUT()   UEP1_CTRL = (UEP1_CTRL & ~MASK_UEP_R_RES) | UEP_R_RES_NAK
#define NAK_EP2_OUT()   UEP2_CTRL = (UEP2_CTRL & ~MASK_UEP_R_RES) | UEP_R_RES_NAK
#define NAK_EP3_OUT()   UEP3_CTRL = (UEP3_CTRL & ~MASK_UEP_R_RES) | UEP_R_RES_NAK
#define NAK_EP4_OUT()   UEP4_CTRL = (UEP4_CTRL & ~MASK_UEP_R_RES) | UEP_R_RES_NAK
                                                               
#define STALL_EP1_OUT() UEP1_CTRL |= UEP_R_RES_STALL
#define STALL_EP2_OUT() UEP2_CTRL |= UEP_R_RES_STALL
#define STALL_EP3_OUT() UEP3_CTRL |= UEP_R_RES_STALL
#define STALL_EP4_OUT() UEP4_CTRL |= UEP_R_RES_STALL

// same as above but using replacement '##'
// works on EP0 too 
#define STALL_IN(ep)  (UEP ##ep ##_CTRL |= UEP_T_RES_STALL)
#define STALL_OUT(ep) (UEP ##ep ##_CTRL |= UEP_R_RES_STALL)

#define NAK_IN(ep)    (UEP ##ep ##_CTRL = (UEP ##ep ##_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_NAK)
#define NAK_OUT(ep)   (UEP ##ep ##_CTRL = (UEP ##ep ##_CTRL & ~MASK_UEP_R_RES) | UEP_R_RES_NAK)

#define ACK_IN(ep)    (UEP ##ep ##_CTRL &= ~MASK_UEP_T_RES) 
#define ACK_OUT(ep)   (UEP ##ep ##_CTRL &= ~MASK_UEP_R_RES)
//prepare a IN Ep for sending data to host
#define ARM_EP(ep,size) {UEP ##ep ##_T_LEN=size;ACK_IN(ep);}
//#define ZLP_EP(ep)    {UEP ##ep ##_T_LEN=0;ACK_OUT(ep);} 
#endif