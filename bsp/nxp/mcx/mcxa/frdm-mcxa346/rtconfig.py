import os
import sys

# toolchains options
ARCH='arm'
CPU='cortex-m33'
CROSS_TOOL='gcc'
BOARD_NAME = 'frdm-mcxa346'
BSP_LIBRARY_TYPE = 'MCXA346'

if os.getenv('RTT_CC'):
    CROSS_TOOL = os.getenv('RTT_CC')
if os.getenv('RTT_ROOT'):
    RTT_ROOT = os.getenv('RTT_ROOT')

# cross_tool provides the cross compiler
# EXEC_PATH is the compiler execute path, for example, CodeSourcery, Keil MDK, IAR
if  CROSS_TOOL == 'gcc':
    PLATFORM    = 'gcc'
    # RT-Thread Env for Windows (v2.0.0) ships gcc-arm-none-eabi-10.3-2021.10.
    # Overridden by RTT_EXEC_PATH, which env-init.bat sets to the same path.
    EXEC_PATH   = r'D:/NXP/env-windows/tools/gnu_gcc/arm_gcc/mingw/bin'
elif CROSS_TOOL == 'keil':
    PLATFORM    = 'armclang'
    EXEC_PATH   = r'C:/Keil_v5'
elif CROSS_TOOL == 'iar':
    PLATFORM    = 'iccarm'
    EXEC_PATH   = r'C:\Program Files\IAR Systems\Embedded Workbench 9.1'

if os.getenv('RTT_EXEC_PATH'):
    EXEC_PATH = os.getenv('RTT_EXEC_PATH')

BUILD = 'debug'
#BUILD = 'release'

if PLATFORM == 'gcc':
    PREFIX = 'arm-none-eabi-'
    CC  = PREFIX + 'gcc'
    CXX = PREFIX + 'g++'
    AS = PREFIX + 'gcc'
    AR = PREFIX + 'ar'
    LINK = PREFIX + 'gcc'
    TARGET_EXT = 'elf'
    SIZE = PREFIX + 'size'
    OBJDUMP = PREFIX + 'objdump'
    OBJCPY = PREFIX + 'objcopy'
    STRIP = PREFIX + 'strip'

    DEVICE = ' -mcpu=' + CPU + ' -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections'
    # __START and __STARTUP_CLEAR_BSS are consumed by the *C* startup file
    # (packages/nxp-mcx-series-latest/MCXA346/startup_MCXA346.c).  Unlike the
    # other MCX parts, MCXA346 has no gcc/startup_MCXA346.S - its gcc/ folder
    # only ships linker scripts - so these defines must be in CFLAGS.  Leaving
    # them in AFLAGS alone means Reset_Handler() falls through to newlib's
    # _start()/_mainCRTStartup() and calls main() directly, never entry(), so
    # rtthread_startup()/rt_hw_board_init() never run and BSS is never zeroed:
    #   -D__START=entry          Reset_Handler() -> entry() (RT-Thread startup)
    #   -D__STARTUP_CLEAR_BSS    zero .bss before RT-Thread touches it
    CFLAGS = DEVICE + ' -Wall -D__FPU_PRESENT -D__START=entry -D__STARTUP_CLEAR_BSS'
    AFLAGS = ' -c' + DEVICE + ' -x assembler-with-cpp -D__START=entry -D__STARTUP_CLEAR_BSS'
    LFLAGS = DEVICE + ' -specs=nano.specs -specs=nosys.specs -Wl,--defsym=__heap_size__=0x8000,--gc-sections,-Map=rtthread.map,--print-memory-usage -Tboard/linker_scripts/MCXA346_flash.ld'

    CPATH = ''
    LPATH = ''

    if BUILD == 'debug':
        CFLAGS += ' -gdwarf-2'
        AFLAGS += ' -gdwarf-2'
        CFLAGS += ' -O0'
    else:
        CFLAGS += ' -O2 -Os'

    # No --remove-section here: .boot_data/.image_vector_table/.ncache are i.MX RT
    # sections and do not exist in MCXA346_flash.ld, so those flags were no-ops.
    # The ihex step matters: without it a GCC build leaves behind whatever
    # rtthread.hex the last Keil build produced, and that stale file is a
    # "flash the wrong firmware" trap (mkdist.py does not exclude *.hex).
    POST_ACTION = OBJCPY + ' -O binary $TARGET rtthread.bin\n' + \
                  OBJCPY + ' -O ihex $TARGET rtthread.hex\n' + SIZE + ' $TARGET \n'

    # module setting 
    CXXFLAGS = ' -Woverloaded-virtual -fno-exceptions -fno-rtti'
    CXXFLAGS += CFLAGS

    M_CFLAGS = CFLAGS + ' -mlong-calls -fPIC '
    M_CXXFLAGS = CXXFLAGS + ' -mlong-calls -fPIC'
    M_LFLAGS = DEVICE + CXXFLAGS + ' -Wl,--gc-sections,-z,max-page-size=0x4' +\
                                    ' -shared -fPIC -nostartfiles -static-libgcc'
    M_POST_ACTION = STRIP + ' -R .hash $TARGET\n' + SIZE + ' $TARGET \n'

elif PLATFORM == 'armcc':
    # toolchains
    CC = 'armcc'
    CXX = 'armcc'
    AS = 'armasm'
    AR = 'armar'
    LINK = 'armlink'
    TARGET_EXT = 'axf'

    DEVICE = ' --cpu ' + CPU + '.fp.sp'
    CFLAGS = DEVICE + ' --apcs=interwork'
    AFLAGS = DEVICE
    LFLAGS = DEVICE + ' --libpath "' + EXEC_PATH + '/ARM/ARMCC/lib" --info sizes --info totals --info unused --info veneers --list rtthread.map --scatter "./MCXA346_flash.scf" '

    LFLAGS += ' --keep *.o(.rti_fn.*)   --keep *.o(FSymTab) --keep *.o(VSymTab)' 

    CFLAGS += ' --diag_suppress=66,1296,186,6134'
    CFLAGS += ' -I' + EXEC_PATH + '/ARM/RV31/INC'
    LFLAGS += ' --libpath ' + EXEC_PATH + '/ARM/RV31/LIB'

    EXEC_PATH += '/arm/bin40/'

    if BUILD == 'debug':
        CFLAGS += ' -g -O0'
        AFLAGS += ' -g'
    else:
        CFLAGS += ' -O2'

    CXXFLAGS = CFLAGS
    CFLAGS += ' --c99'

    POST_ACTION = 'fromelf -z $TARGET'
    # POST_ACTION = 'fromelf --bin $TARGET --output rtthread.bin \nfromelf -z $TARGET'

elif PLATFORM == 'armclang':
    # toolchains
    CC = 'armclang'
    CXX = 'armclang'
    AS = 'armasm'
    AR = 'armar'
    LINK = 'armlink'
    TARGET_EXT = 'axf'

    DEVICE = ' --cpu Cortex-M33 '
    CFLAGS = ' --target=arm-arm-none-eabi'
    CFLAGS += ' -mcpu=' + CPU
    CFLAGS += ' -c -fno-rtti -funsigned-char -fshort-enums -fshort-wchar '
    CFLAGS += ' -gdwarf-3 -ffunction-sections '
    AFLAGS = DEVICE + ' --apcs=interwork '
    LFLAGS = DEVICE + ' --info sizes --info totals --info unused --info veneers '
    LFLAGS += ' --list rt-thread.map '
    LFLAGS += r' --strict --scatter "board\linker_scripts\MCXA346_flash.sct" '
    CFLAGS += ' -I' + EXEC_PATH + '/ARM/ARMCLANG/include'
    LFLAGS += ' --libpath=' + EXEC_PATH + '/ARM/ARMCLANG/lib'

    EXEC_PATH += '/ARM/ARMCLANG/bin/'

    if BUILD == 'debug':
        CFLAGS += ' -g -O1' # armclang recommend
        AFLAGS += ' -g'
    else:
        CFLAGS += ' -O2'
        
    CXXFLAGS = CFLAGS
    CFLAGS += ' -std=c99'

    POST_ACTION = 'fromelf --bin $TARGET --output rtthread.bin \nfromelf -z $TARGET'

elif PLATFORM == 'iccarm':
    CC = 'iccarm'
    CXX = 'iccarm'
    AS = 'iasmarm'
    AR = 'iarchive'
    LINK = 'ilinkarm'
    TARGET_EXT = 'out'

    DEVICE = ' -D__FPU_PRESENT'

    CFLAGS = DEVICE
    CFLAGS += ' --diag_suppress Pa050'
    CFLAGS += ' --no_cse'
    CFLAGS += ' --no_unroll'
    CFLAGS += ' --no_inline'
    CFLAGS += ' --no_code_motion'
    CFLAGS += ' --no_tbaa'
    CFLAGS += ' --no_clustering'
    CFLAGS += ' --no_scheduling'
    CFLAGS += ' --debug'
    CFLAGS += ' --endian=little'
    CFLAGS += ' --cpu=' + CPU
    CFLAGS += ' -e'
    CFLAGS += ' --fpu=None'
    CFLAGS += ' --dlib_config "' + EXEC_PATH + '/arm/INC/c/DLib_Config_Normal.h"'
    CFLAGS += ' -Ol'
    CFLAGS += ' --use_c++_inline'

    AFLAGS = ''
    AFLAGS += ' -s+'
    AFLAGS += ' -w+'
    AFLAGS += ' -r'
    AFLAGS += ' --cpu ' + CPU
    AFLAGS += ' --fpu None'

    if BUILD == 'debug':
        CFLAGS += ' --debug'
        CFLAGS += ' -On'
    else:
        CFLAGS += ' -Oh'

    LFLAGS = ' --config "board/linker_scripts/MCXA346_flash.icf"'
    LFLAGS += ' --redirect _Printf=_PrintfTiny'
    LFLAGS += ' --redirect _Scanf=_ScanfSmall'
    LFLAGS += ' --entry __iar_program_start'

    CXXFLAGS = CFLAGS

    EXEC_PATH = EXEC_PATH + '/arm/bin/'
    POST_ACTION = 'ielftool --bin $TARGET rtthread.bin'

def dist_handle(BSP_ROOT, dist_dir):
    cwd_path = os.getcwd()
    sys.path.append(os.path.join(os.path.dirname(BSP_ROOT), '..', 'tools'))
    from sdk_dist import dist_do_building
    dist_do_building(BSP_ROOT, dist_dir)
    
