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

/*
    A simple RTOS thread kernel

    Author: Andrew Somerville <andy16666@gmail.com> 
    GitHub: andy16666
 */
#ifndef THREADKRNEL_HH
#define THREADKRNEL_HH
#define threadkernel_t struct threadkernel_t_t
#define process_t struct process_t_t
#include<sys/types.h>
#include<stdint.h>

// Constructor 
threadkernel_t* create_threadkernel(unsigned long (*millis)());

struct threadkernel_t_t {
  unsigned long (*millis)(); 
  process_t *processes;

  void  (*add) (threadkernel_t *k, void (*f)(), unsigned long periodMilliseconds);
  void  (*run) (threadkernel_t *k);
};

struct process_t_t {
  unsigned long periodMilliseconds; 
  unsigned long nextRunMilliseconds; 
  process_t *next; 

  void (*f)(); 
}; 

void __threadkernel_add(threadkernel_t *k, void (*f)(), unsigned long periodMilliseconds);
void __threadkernel_run(threadkernel_t *k);

#endif 