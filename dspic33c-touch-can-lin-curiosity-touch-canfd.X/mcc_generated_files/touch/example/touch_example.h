#ifndef TOUCH_EXAMPLE_H
#define	TOUCH_EXAMPLE_H

#ifdef	__cplusplus
extern "C" {
#endif

#include"../touch.h" 
extern volatile uint8_t measurement_done_touch;   

void touch_mainloop_example(void);
void touch_status_display(void);

#ifdef	__cplusplus
}
#endif

#endif	/* TOUCH_EXAMPLE_H */