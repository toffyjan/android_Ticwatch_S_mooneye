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

#ifndef __DDP_CMDQ_H__
#define __DDP_CMDQ_H__


#define CMDQ_BUFFER_NUM 8
#define CMDQ_BUFFER_SIZE 32*1024

#define CMDQ_ION_BUFFER_SIZE CMDQ_BUFFER_SIZE*CMDQ_BUFFER_NUM

#define MAX_CMDQ_TASK_ID 256

#define CMDQ_THREAD_NUM 7
#define CMDQ_THREAD_LIST_LENGTH 10
#define CMDQ_TIMEOUT 27


typedef struct {
    int Owner;
    unsigned long VA;
    unsigned long MVA;
    unsigned int blocksize;
    unsigned long *blockTailAddr;
} cmdq_buff_t;


typedef struct {
    int cmdBufID;
    int cmdqThread;
} task_resource_t;



enum
{
    /* CAM */
    tIMGI   = 0,
    tIMGO   = 1,
    tIMG2O  = 2,

    /* MDP */
    tRDMA0  = 3,
    tCAMIN  = 4,
    tSCL0   = 5,
    tSCL1   = 6,
    tTDSHP  = 7,
    tWROT   = 8,
    tWDMA1  = 9,

    tTotal  = 10,
};


void cmdqBufferTbl_init(unsigned long va_base, unsigned long mva_base);
int cmdqResource_required(void);
void cmdqResource_free(int taskID);
cmdq_buff_t *cmdqBufAddr(int taskID);
bool cmdqTaskAssigned(int taskID, unsigned int priority, unsigned int engineFlag, unsigned int blocksize);
void cmdqThreadComplete(int cmdqThread, bool cmdqIntStatus);
void dumpMDPRegInfo(void);
void cmdqTerminated(void);
bool checkMdpEngineStatus(unsigned int engineFlag);




#endif
