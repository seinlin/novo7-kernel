#ifndef _PPMGR_PRI_INCLUDE__
#define _PPMGR_PRI_INCLUDE__

/* for ppmgr device op. */
extern int  init_ppmgr_device(void);
extern int uninit_ppmgr_device(void);

/* for ppmgr device class op. */
extern struct class* init_ppmgr_cls(void);

/* for thread of ppmgr. */
extern int start_vpp_task(void);
extern void stop_vpp_task(void);

/* for ppmgr private member. */
extern void set_ppmgr_buf_info(char* start,unsigned int size);
extern void get_ppmgr_buf_info(char** start,unsigned int* size);

/*  ppmgr buffer op. */
extern int ppmgr_buffer_init(void);

#endif /* _PPMGR_PRI_INCLUDE__ */
