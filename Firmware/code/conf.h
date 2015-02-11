/**
  ******************************************************************************
  * @file    C_HEADER_TEMPLATE.h
  * @author  Header template author and its contact email
  * @version Version 1 Issue 1
  * @date    Date when the file was released.	(i.e. 05-March-2014)
  * @brief   Short description about the header template.
  ******************************************************************************
  * @attention
  *
  *		Copyright Information (C)
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CONF_H
#define __CONF_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
#define	USE_FULL_ASSERT	1
#define	DEBUG			1

/* Exported macro ------------------------------------------------------------*/
#ifdef	USE_FULL_ASSERT
#	define	ASSERT_PARAM(x)		( (x) ? (void)0 : assert_failed(__FILE__, __LINE__) )
#else
#	define	ASSERT_PARAM(x)		( (void)0 )
#endif

/* Exported functions --------------------------------------------------------*/

void assert_failed (char* file, uint8_t line)
{
	printf("> ASSERT FAILED: Wrong parameter at file %s on line %i.\n", file, line);

	while (1)
	{

	}
}

#ifdef __cplusplus
}
#endif

#endif /*__CONF_H */
 
