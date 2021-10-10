#ifndef REAR_CAM_DRV_H
#define REAR_CAM_DRV_H

#define RCAM_AFTER_BOOT         0
#define RCAM_COLD_BOOT          1

#define	RCAM_ST_RELEASE_VIDEO   (0x01 << 16)
#define	RCAM_ST_RUN             (0x01 << 17)	
#define	RCAM_ST_DIRECT_SHOW     (0x01 << 18)	
#define	RCAM_ST_PARKING_LINE    (0x01 << 19)	
#define RCAM_VALID_HEADER       0xFEAA005C


#define USING_EARLY_CAM_CM4
#include "tcc_rear_cam_ioctl.h"

struct rear_cam_reg
{
    // reg
    void __iomem *in_wmix_cam;
    void __iomem *in_wmix_rdma;
    void __iomem *in_viqe;
    void __iomem *in_scaler;
    void __iomem *in_wmix;
    void __iomem *in_wmix_wdma;
    void __iomem *lcd_wmix;
    void __iomem *lcd_wmix_rdma;
    void __iomem *vioc_config;
    void __iomem *camera_port;
    
    //void __iomem *
};

struct rear_cam_soc
{
        struct rear_cam_reg reg; 

        struct device *dev;

        // IRQ
        unsigned int in_wmix_wdma_irq;
        unsigned int gear_port_irq;

        // WDMA INTR
        int wdma_wakeup_int;
        wait_queue_head_t wdma_isr_wait;

        // GEAR PORT
        unsigned int gear_port;
        unsigned int gear_port_active_level;

        // CIF PORT
        unsigned int camera_port;
        
        // MISC        
        unsigned int mStatus;
        
        unsigned int mFileOpenCount;

        unsigned int mRearEventStatus;

	 // lcd panel
	 unsigned int lcd_width;
	 unsigned int lcd_height;
	 
        // rear cam preview
        unsigned int preview_x;						// vin position.x in wmix
        unsigned int preview_y;						// vin position.y in wmix

	 unsigned int preview_crop_x;					// rear cam position.x in scaler
	 unsigned int preview_crop_y;					// rear cam position.y in scaler
	 
        unsigned int preview_width;					
        unsigned int preview_height;
        unsigned int preview_format;
        
        unsigned int preview_additional_width;
        unsigned int preview_additional_height;

	 unsigned int draw_preview_width;
	 unsigned int draw_preview_height;
	
        // rear cam parking line
        unsigned int parking_line_x;
        unsigned int parking_line_y;
        unsigned int parking_line_width;
        unsigned int parking_line_height;
        unsigned int parking_line_format;

	unsigned int draw_parking_line_width;
	unsigned int draw_parking_line_height;
	
		
        // Memory
        unsigned long mViqeMemory;
        unsigned long mPreviewBufAddr[2];
        unsigned long mLineBufAddr[2];
        unsigned long mLineBufIndex;
        unsigned long mLineBufEnabled;
        
        unsigned long mCurrAliveCount;
        unsigned long mAliveThreadRun;
        unsigned long mPrevAliveBase;

        unsigned long mSuspend;
        
        // SYNC
        struct task_struct *mAliveThread;
        struct mutex mMutex;
        struct mutex mShowMutex;
        
};

extern void RCAM_DeInitialize(void);
extern unsigned long RCAM_Initialize(struct platform_device *pdev);
extern void RCAM_OnOff(long OnOff);
extern void RCAM_LINE_OnOff(long OnOff);
extern void RCAM_ReleaseVideo(void);
//extern void RCAM_LineBufUpdate(unsigned long bufnum);
extern void RCAM_LineBufUpdate(RCAM_LINE_BUFFER_UPDATE_INFO * pInfo);
extern	void RCAM_WaitEvent(void);
extern	void RCAM_SetRearEvent(int OnOff);
extern	void RCAM_SetPreviewVIOC(void);
extern	void RCAM_PreviewUpdate(unsigned int buf_address);
extern  void RCAM_IsPreviewShow(unsigned int booting);
extern void RCAM_CheckAlive(void);
extern void RCAM_SetScalerOverscan(unsigned long overscan_position_x, unsigned long overscan_position_y, unsigned long overscan_width_factor, unsigned long overscan_height_factor);
extern unsigned int RCAM_GetHwGearStatus(void);
extern unsigned int RCAM_GetRearViewOvp(void);

extern struct rear_cam_soc *rear_cam_soc;

extern void RCAM_GetLineBufAddrByIndex(RCAM_LINE_BUFFER_INFO * rcam_line_buffer_info);
extern void RCAM_LineSetPosition(RCAM_LINE_POSITION_INFO * rcam_line_position_info);
extern void RCAM_LineSetSize(RCAM_LINE_SIZE_INFO * rcam_line_size_info);
extern void RCAM_PreviewSetPosition(unsigned int preview_position_x, unsigned int preview_position_y);
extern void RCAM_PreviewSetSize(unsigned int width, unsigned int height);

#ifdef USING_EARLY_CAM_CM4
extern int RCAM_Handover(int cm4_gear_status);
#endif

#endif
