#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Check production FRAM multi-block writes with distinct source pages."""
from pathlib import Path
import subprocess, tempfile, sys
root=Path(__file__).resolve().parents[2]
source=Path(sys.argv[1]).read_text() if len(sys.argv)>1 else (root/'platforms/nuttx/NuttX/nuttx/drivers/mtd/ramtron.c').read_text()
def body_after(marker):
    start=source.index(marker); opening=source.index('\n{',start)+1; end=opening+1; depth=1
    while depth:
        depth+=(source[end]=='{')-(source[end]=='}'); end+=1
    return source[opening:end]
plain=body_after(' * Name: ramtron_bwrite/ramtron_bwrite_nonchunked')
chunked=body_after(' * Name: ramtron_bwrite_chunked')
harness=r'''
#include <cassert>
#include <cstdint>
#include <cstring>
#include <vector>
#include <sys/types.h>
#define FAR
#define finfo(...) ((void)0)
#define DEBUGASSERT(x) assert(x)
struct mtd_dev_s {};
struct ramtron_parts_s { uint32_t chunksize; };
struct ramtron_dev_s { mtd_dev_s mtd; uint8_t pageshift=5; ramtron_parts_s *part; int dev=0; };
static std::vector<uint8_t> memory(32768,0xee);
static bool locked=false;
static int calls=0,fail_call=-1;
void ramtron_lock(ramtron_dev_s*){assert(!locked);locked=true;}
void ramtron_unlock(int){assert(locked);locked=false;}
int ramtron_pagewrite(ramtron_dev_s*,const uint8_t *buffer,off_t page,size_t size){
 assert(locked);if(calls++==fail_call)return -1;
 assert(page>=0 && page*size+size<=memory.size());
 memcpy(memory.data()+page*size,buffer,size);return 0;
}
static ssize_t plain(mtd_dev_s *dev,off_t startblock,size_t nblocks,const uint8_t *buffer)
'''+plain+r'''
static ssize_t chunked(mtd_dev_s *dev,off_t startblock,size_t nblocks,const uint8_t *buffer)
'''+chunked+r'''
int main(){
 ramtron_parts_s part{8};ramtron_dev_s dev;dev.part=&part;
 for(auto fn:{plain,chunked}){
  for(unsigned blocks:{0,1,2,3,17,51}){
   std::vector<uint8_t> input(blocks*32+1);
   for(unsigned i=0;i<input.size();++i)input[i]=static_cast<uint8_t>((i*7+i/32*13)%251);
   std::fill(memory.begin(),memory.end(),0xee);calls=0;fail_call=-1;
   assert(fn(&dev.mtd,3,blocks,input.data())==blocks);
   assert(!locked);
   assert(memcmp(memory.data()+96,input.data(),blocks*32)==0);
   assert(memory[95]==0xee && memory[96+blocks*32]==0xee);
  }
  std::vector<uint8_t> input(96,0xaa);calls=0;fail_call=1;
  assert(fn(&dev.mtd,0,3,input.data())==0 && !locked);
 }
}
'''
with tempfile.TemporaryDirectory() as tmp:
    src=Path(tmp)/'check.cpp'; src.write_text(harness); binary=Path(tmp)/'check'
    subprocess.run(['g++','-std=c++17','-Wall','-Wextra','-Werror',str(src),'-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)
print('PASS: production FRAM plain/chunked write bodies preserve all blocks at nonzero offsets; 0/1/2/3/17/51 blocks, boundaries and error unlock')
