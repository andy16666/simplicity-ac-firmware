/*
 * This program is free software: you can redistribute it and/or modify it 
 * under the terms of the GNU General Public License as published by the 
 * Free Software Foundation, either version 3 of the License, or (at your 
 * option) any later version.
 * 
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY 
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for 
 * more details.
 * 
 * You should have received a copy of the GNU General Public License along 
 * with this program. If not, see <https://www.gnu.org/licenses/>.
 */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "threadkernel.h"

threadkernel_t* create_threadkernel(unsigned long(*millis)()) {
  threadkernel_t* k = (threadkernel_t *)malloc(sizeof(threadkernel_t)); 

  k->millis    = millis; 
  k->processes = 0; 
  k->add       = __threadkernel_add; 
  k->run       = __threadkernel_run; 

  return k;
}

void __threadkernel_add(threadkernel_t *k, void(*f)(), unsigned long periodMilliseconds) 
{
  process_t** process_ptr = &(k->processes); 
  while(*process_ptr) process_ptr = &((*process_ptr)->next); 
  *process_ptr = (process_t*)malloc(sizeof(process_t)); 
  (*process_ptr)->periodMilliseconds = periodMilliseconds; 
  (*process_ptr)->nextRunMilliseconds = 0; 
  (*process_ptr)->next = 0; 
  (*process_ptr)->f = f; 
}

void __threadkernel_run(threadkernel_t *k)
{
  process_t *process = k->processes; 
  
  while(process)
  {
    unsigned long timeMilliseconds = k->millis();  

    if (timeMilliseconds >= process->nextRunMilliseconds)
    {
      process->f(); 
      process->nextRunMilliseconds = timeMilliseconds + process->periodMilliseconds; 
    }
    
    process = process->next; 
  }
}