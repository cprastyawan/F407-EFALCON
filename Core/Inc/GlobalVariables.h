/*
 * GlobalVariables.h
 *
 *  Created on: Aug 25, 2020
 *      Author: LENOVO
 */

#ifndef INC_GLOBALVARIABLES_H_
#define INC_GLOBALVARIABLES_H_

#define map(x,in_min,in_max,out_min,out_max) ( (x-in_min) * (out_max-out_min) / (in_max-in_min) + out_min )
#define constrain(nilaix,bawah,atas) ( (nilaix)<(bawah) ? (bawah) : ( (nilaix)>(atas) ? (atas) : (nilaix) ) )

#endif /* INC_GLOBALVARIABLES_H_ */
