/*
* Copyright (C) 2011-2014 MediaTek Inc.
*
* This program is free software: you can redistribute it and/or modify it under the terms of the
* GNU General Public License version 2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
* without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
* See the GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with this program.
* If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef __CUST_EINTH
#define __CUST_EINTH
#ifdef __cplusplus
extern "C" {
#endif
#define CUST_EINT_POLARITY_LOW              0
#define CUST_EINT_POLARITY_HIGH             1
#define CUST_EINT_DEBOUNCE_DISABLE          0
#define CUST_EINT_DEBOUNCE_ENABLE           1
#define CUST_EINT_EDGE_SENSITIVE            0
#define CUST_EINT_LEVEL_SENSITIVE           1
/* //////////////////////////////////////////////////////////////////////////// */


#define CUST_EINT_SENSORHUB_NUM              5
#define CUST_EINT_SENSORHUB_DEBOUNCE_CN      0
#define CUST_EINT_SENSORHUB_POLARITY         CUST_EINT_POLARITY_HIGH
#define CUST_EINT_SENSORHUB_SENSITIVE        CUST_EINT_EDGE_SENSITIVE
#define CUST_EINT_SENSORHUB_DEBOUNCE_EN      CUST_EINT_DEBOUNCE_DISABLE

#define CUST_EINT_TOUCH_PANEL_NUM              1
#define CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN      0
#define CUST_EINT_TOUCH_PANEL_POLARITY         CUST_EINT_POLARITY_LOW
#define CUST_EINT_TOUCH_PANEL_SENSITIVE        CUST_EINT_EDGE_SENSITIVE
#define CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN      CUST_EINT_DEBOUNCE_DISABLE

#define CUST_EINT_WIFI_NUM              13
#define CUST_EINT_WIFI_DEBOUNCE_CN      10
#define CUST_EINT_WIFI_POLARITY         CUST_EINT_POLARITY_LOW
#define CUST_EINT_WIFI_SENSITIVE        CUST_EINT_LEVEL_SENSITIVE
#define CUST_EINT_WIFI_DEBOUNCE_EN      CUST_EINT_DEBOUNCE_DISABLE




/* //////////////////////////////////////////////////////////////////////////// */
#ifdef __cplusplus
}
#endif
#endif				/* _CUST_EINT_H */
