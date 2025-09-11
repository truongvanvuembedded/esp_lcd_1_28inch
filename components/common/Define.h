
#ifndef  ASTIDEF
#define ASTIDEF

//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
typedef	unsigned char			U1 ;
typedef unsigned short			U2 ;
typedef unsigned long			U4 ;

typedef	signed char				S1 ;
typedef signed short			S2 ;
typedef signed long				S4 ;

typedef volatile unsigned char 	VU1 ;
typedef volatile unsigned short VU2 ;
typedef volatile unsigned long	VU4 ;

typedef volatile signed char	VS1 ;
typedef volatile signed short	VS2 ;
typedef volatile signed long	VS4 ;

//------------------------------------------------------------------------------
//	��{�萔�錾
//------------------------------------------------------------------------------
#define		U1TRUE			((U1)1)
#define		U1FALSE			((U1)0)
#define		U1ON			((U1)1)
#define		U1OFF			((U1)0)
#define		U1HI			((U1)1)
#define		U1LO			((U1)0)
#define		U1UP			((U1)1)
#define		U1DOWN			((U1)0)
#define		U1OK			((U1)1)
#define		U1NG			((U1)0)

#define		BIT_OFF			((U1)0x00)
#define		BIT_ON			((U1)0x01)

#define		U1MAX			((U1)0xff)
#define		U2MAX			((U2)0xffff)
#define		U4MAX			((U4)0xffffffff)

#define		U1MIN			((U1)0x00)
#define		U2MIN			((U2)0x0000)
#define		U4MIN			((U4)0x00000000)

#define		S1MAX			((S1)0x7f)
#define		S2MAX			((S2)0x7fff)
#define		S4MAX			((S4)0x7fffffff)

#define		S1MIN			((S1)0x80)
#define		S2MIN			((S2)0x8000)
#define		S4MIN			((S4)0x80000000)

#define		S1ZERO			((S1)0x00)
#define		S2ZERO			((S2)0x0000)
#define		S4ZERO			((S4)0x00000000)

#define 	U1_HI_NBL	 	((U1)0xf0)
#define 	U1_LO_NBL	 	((U1)0x0f)

#endif
/*====================================== End of File =============================================*/
