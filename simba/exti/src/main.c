
#include "simba.h"

static volatile int flag = 0;

static void isr(void *arg_p)
{
    flag++;
    //std_printf(FSTR("flag++ = %d\r\n"), (int)flag);
}

int test_exti(void)
{
    int i;
    struct exti_driver_t exti;
    struct pin_driver_t pin;

    pin_init(&pin, &pin_D4_dev, PIN_OUTPUT);
    pin_write(&pin, 1);

    BTASSERT(exti_init(&exti,
                       &exti_D1_dev, // ADDED FROM MCU C FILE
                       EXTI_TRIGGER_FALLING_EDGE,
                       isr,
                       NULL) == 0);
    BTASSERT(exti_start(&exti) == 0);

    thrd_sleep_ms(5000);

    #define TRIGCNT (10)
    for (i = 0; i < TRIGCNT; i++) {
        pin_write(&pin, 0);
        time_busy_wait_us(10000);
        pin_write(&pin, 1);
        time_busy_wait_us(10000);
        //std_printf(FSTR("toggle\r\n"));
    }

    std_printf(FSTR("flag = %d\r\n"), (int)flag);
    BTASSERT(flag == 10);

    while(1)
    {
      std_printf(FSTR("flag = %d\r\n"), (int)flag);
      thrd_sleep_ms(500);
      //time_busy_wait_us(1000000);
    }

    return (0);
}

int main()
{
    // struct harness_testcase_t harness_testcases[] = {
    //     { test_exti, "test_exti" },
    //     { NULL, NULL }
    // };

    sys_start();
    exti_module_init();

    test_exti();

    //harness_run(harness_testcases);

    return (0);
}
