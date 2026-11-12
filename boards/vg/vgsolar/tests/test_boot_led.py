"""Run actual VGSolar LED C code with a mock HRT/GPIO and the board hook in bash."""
from pathlib import Path
import os
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[4]
source = (ROOT / 'boards/vg/vgsolar/src/led.c').read_text()
source = re.sub(r'^#include .*$', '', source, flags=re.M)
stub = r'''
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define __EXPORT
#define GPIO_nLED_BLUE 1
#define GPIO_nLED_RED 2
typedef unsigned irqstate_t;
struct hrt_call { void (*callback)(void *); };
static bool gpio[3] = {true, true, true};
static void stm32_configgpio(uint32_t pin) { (void)pin; }
static void stm32_gpiowrite(uint32_t pin, bool high) { gpio[pin] = high; }
static bool stm32_gpioread(uint32_t pin) { return gpio[pin]; }
static irqstate_t enter_critical_section(void) { return 0; }
static void leave_critical_section(irqstate_t flags) { (void)flags; }
static void hrt_cancel(struct hrt_call *call) { call->callback = NULL; }
static void hrt_call_every(struct hrt_call *call, unsigned delay, unsigned period,
                          void (*callback)(void *), void *arg) {
    assert(delay == 125000 && period == 125000); (void)arg;
    call->callback = callback;
}
'''
checks = r'''
static void tick(void) { assert(boot_led_call.callback); boot_led_call.callback(NULL); }
int main(void) {
    vgsolar_boot_led_start();
    assert(gpio[GPIO_nLED_BLUE] && !gpio[GPIO_nLED_RED]);
    assert(!vgsolar_boot_led_completed() && !vgsolar_boot_led_failed());
    tick(); assert(gpio[GPIO_nLED_RED]);
    tick(); assert(gpio[GPIO_nLED_RED]);
    tick(); assert(!gpio[GPIO_nLED_RED]);
    tick(); assert(!gpio[GPIO_nLED_RED]);
    vgsolar_boot_led_complete();
    assert(vgsolar_boot_led_completed() && !boot_led_call.callback);
    assert(!gpio[GPIO_nLED_BLUE] && gpio[GPIO_nLED_RED]);
    vgsolar_boot_led_fail();
    assert(vgsolar_boot_led_failed() && !vgsolar_boot_led_completed());
    assert(gpio[GPIO_nLED_BLUE] && !gpio[GPIO_nLED_RED]);
    tick(); assert(gpio[GPIO_nLED_RED]);
    tick(); assert(!gpio[GPIO_nLED_RED]);
    vgsolar_boot_led_complete();
    assert(vgsolar_boot_led_failed() && !vgsolar_boot_led_completed());
    assert(gpio[GPIO_nLED_BLUE] && boot_led_call.callback);
    vgsolar_boot_led_start();
    assert(!vgsolar_boot_led_failed() && !vgsolar_boot_led_completed());
    vgsolar_boot_led_fail();
    vgsolar_boot_led_fail();
    vgsolar_boot_led_complete();
    assert(gpio[GPIO_nLED_BLUE] && boot_led_call.callback);
    return 0;
}
'''
with tempfile.TemporaryDirectory() as td:
    c = Path(td) / 'led.c'
    c.write_text(stub + source + checks)
    exe = Path(td) / 'led_test'
    subprocess.run(['cc', '-std=c11', '-Wall', '-Wextra', '-Werror', str(c), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
print('PASS actual LED backend: slow red, steady blue, fast red, failure latch, restart')

hook = (ROOT / 'boards/vg/vgsolar/init/rc.board_boot_complete').read_text()
mock = r'''
commander() { [ "$FAIL_SERVICE" != commander ]; }
sensors() { [ "$FAIL_SERVICE" != sensors ]; }
param() { [ "$FAIL_SERVICE" != usb_disabled ]; }
cdcacm_autostart() { [ "$FAIL_SERVICE" != usb_manager ]; }
led_control() {
    echo "LED_COMMAND=$1"
    if [ "$1" = boot_failed_wait ]; then exit 20; fi
    if [ "$1" = boot_complete_wait ]; then exit 10; fi
    exit 99
}
'''
for label, storage, vehicle, failure, expected in [
    ('TF missing', 'no', 'rover', '', 20),
    ('TF mount failed', 'no', 'rover', '', 20),
    ('airframe missing', 'yes', 'none', '', 20),
    ('commander absent', 'yes', 'rover', 'commander', 20),
    ('sensors absent', 'yes', 'rover', 'sensors', 20),
    ('USB disabled', 'yes', 'rover', 'usb_disabled', 20),
    ('USB manager absent', 'yes', 'rover', 'usb_manager', 20),
    ('startup checks pass', 'yes', 'rover', '', 10),
]:
    env = dict(os.environ, STORAGE_AVAILABLE=storage, VEHICLE_TYPE=vehicle, FAIL_SERVICE=failure)
    run = subprocess.run(['bash', '-c', mock + hook], env=env, capture_output=True, text=True)
    assert run.returncode == expected, (label, run.returncode, run.stdout, run.stderr)
    assert run.stdout.count('LED_COMMAND=') == 1, run.stdout
    print('PASS board hook:', label)

rcs = (ROOT / 'ROMFS/px4fmu_common/init.d/rcS').read_text()
assert rcs.index('mavlink boot_complete') < rcs.index('. $BOARD_RC_BOOT_COMPLETE')
for var in ('STORAGE_AVAILABLE', 'VEHICLE_TYPE'):
    assert rcs.index('. $BOARD_RC_BOOT_COMPLETE') < rcs.index('unset ' + var)
header = (ROOT / 'boards/vg/vgsolar/src/board_config.h').read_text()
assert '#define BOARD_OVERLOAD_LED ' not in header
print('PASS completion hook ordering, preserved inputs, exclusive red LED ownership')
