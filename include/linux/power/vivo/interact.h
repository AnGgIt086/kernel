#ifndef __INTERACT_H
#define __INTERACT_H

#include <linux/slab.h>
#include <linux/types.h>
#include <linux/reboot.h>
#include <linux/power/vivo/power_supply_lite.h>
#include <mt-plat/mtk_boot_common.h>
#ifdef CONFIG_VIVO_CHARGING_FOR_PD2226
#include <mach/upmu_hw.h>
#include <mach/upmu_sw.h>
#include <mach/mtk_pmic.h>
#include <mach/mtk_battery_property.h>
#include <mt-plat/mtk_boot.h>
#endif


/*=============================================================================
 * PMIC IRQ ENUM define
 *=============================================================================
 */
enum PMIC_IRQ_ENUM {
	INT_VPU_OC,
	SP_BUCK_TOP_START = INT_VPU_OC,
	INT_VCORE_OC,
	INT_VGPU11_OC,
	INT_VGPU12_OC,
	INT_VMODEM_OC,
	INT_VPROC1_OC,
	INT_VPROC2_OC,
	INT_VS1_OC,
	INT_VS2_OC,
	INT_VPA_OC,
	NO_USE_0_10,
	NO_USE_0_11,
	NO_USE_0_12,
	NO_USE_0_13,
	NO_USE_0_14,
	NO_USE_0_15,
	INT_VFE28_OC,
	SP_LDO_TOP_START = INT_VFE28_OC,
	INT_VXO22_OC,
	INT_VRF18_OC,
	INT_VRF12_OC,
	INT_VEFUSE_OC,
	INT_VCN33_1_OC,
	INT_VCN33_2_OC,
	INT_VCN13_OC,
	INT_VCN18_OC,
	INT_VA09_OC,
	INT_VCAMIO_OC,
	INT_VA12_OC,
	INT_VAUX18_OC,
	INT_VAUD18_OC,
	INT_VIO18_OC,
	INT_VSRAM_PROC1_OC,
	INT_VSRAM_PROC2_OC,
	INT_VSRAM_OTHERS_OC,
	INT_VSRAM_MD_OC,
	INT_VEMC_OC,
	INT_VSIM1_OC,
	INT_VSIM2_OC,
	INT_VUSB_OC,
	INT_VRFCK_OC,
	INT_VBBCK_OC,
	INT_VBIF28_OC,
	INT_VIBR_OC,
	INT_VIO28_OC,
	INT_VM18_OC,
	INT_VUFS_OC,
	NO_USE_2_14,
	NO_USE_2_15,
	INT_PWRKEY,
	SP_PSC_TOP_START = INT_PWRKEY,
	INT_HOMEKEY,
	INT_PWRKEY_R,
	INT_HOMEKEY_R,
	INT_NI_LBAT_INT,
	INT_CHRDET_EDGE,
	NO_USE_3_6,
	NO_USE_3_7,
	NO_USE_3_8,
	NO_USE_3_9,
	NO_USE_3_10,
	NO_USE_3_11,
	NO_USE_3_12,
	NO_USE_3_13,
	NO_USE_3_14,
	NO_USE_3_15,
	INT_RTC,
	SP_SCK_TOP_START = INT_RTC,
	NO_USE_4_1,
	NO_USE_4_2,
	NO_USE_4_3,
	NO_USE_4_4,
	NO_USE_4_5,
	NO_USE_4_6,
	NO_USE_4_7,
	NO_USE_4_8,
	NO_USE_4_9,
	NO_USE_4_10,
	NO_USE_4_11,
	NO_USE_4_12,
	NO_USE_4_13,
	NO_USE_4_14,
	NO_USE_4_15,
	INT_FG_BAT_H,
	SP_BM_TOP_START = INT_FG_BAT_H,
	INT_FG_BAT_L,
	INT_FG_CUR_H,
	INT_FG_CUR_L,
	INT_FG_ZCV,
	NO_USE_5_5,
	NO_USE_5_6,
	INT_FG_N_CHARGE_L,
	INT_FG_IAVG_H,
	INT_FG_IAVG_L,
	NO_USE_5_10,
	INT_FG_DISCHARGE,
	INT_FG_CHARGE,
	NO_USE_5_13,
	NO_USE_5_14,
	NO_USE_5_15,
	INT_BATON_LV,
	NO_USE_6_1,
	INT_BATON_BAT_IN,
	INT_BATON_BAT_OUT,
	INT_BIF,
	NO_USE_6_5,
	NO_USE_6_6,
	NO_USE_6_7,
	NO_USE_6_8,
	NO_USE_6_9,
	NO_USE_6_10,
	NO_USE_6_11,
	NO_USE_6_12,
	NO_USE_6_13,
	NO_USE_6_14,
	NO_USE_6_15,
	INT_BAT_H,
	SP_HK_TOP_START = INT_BAT_H,
	INT_BAT_L,
	INT_BAT2_H,
	INT_BAT2_L,
	INT_BAT_TEMP_H,
	INT_BAT_TEMP_L,
	INT_THR_H,
	INT_THR_L,
	INT_AUXADC_IMP,
	INT_NAG_C_DLTV,
	NO_USE_7_10,
	NO_USE_7_11,
	NO_USE_7_12,
	NO_USE_7_13,
	NO_USE_7_14,
	NO_USE_7_15,
	INT_AUDIO,
	SP_AUD_TOP_START = INT_AUDIO,
	NO_USE_8_1,
	NO_USE_8_2,
	NO_USE_8_3,
	NO_USE_8_4,
	INT_ACCDET,
	INT_ACCDET_EINT0,
	INT_ACCDET_EINT1,
	NO_USE_8_8,
	NO_USE_8_9,
	NO_USE_8_10,
	NO_USE_8_11,
	NO_USE_8_12,
	NO_USE_8_13,
	NO_USE_8_14,
	NO_USE_8_15,
	INT_SPI_CMD_ALERT,
	SP_MISC_TOP_START = INT_SPI_CMD_ALERT,
	NO_USE_9_1,
	NO_USE_9_2,
	NO_USE_9_3,
	NO_USE_9_4,
	NO_USE_9_5,
	NO_USE_9_6,
	NO_USE_9_7,
	NO_USE_9_8,
	NO_USE_9_9,
	NO_USE_9_10,
	NO_USE_9_11,
	NO_USE_9_12,
	NO_USE_9_13,
	NO_USE_9_14,
	NO_USE_9_15,
	INT_ENUM_MAX,
};

/************************************************************
 *
 *   [quote function]
 *
 ***********************************************************/
#ifdef CONFIG_VIVO_CHARGING_FOR_PD2226
extern int pmic_get_auxadc_value(int list);
#endif
//extern int pmic_get_channel_value(u8 list);
//extern int pmic_is_bif_exist(void);
#if 0
extern void pmic_register_interrupt_callback(enum PMIC_IRQ_ENUM intNo, void (EINT_FUNC_PTR) (void));
#else
extern void pmic_register_interrupt_callback_ex(void (EINT_FUNC_PTR) (void));
#endif
extern unsigned int upmu_get_rgs_chrdet(void);
extern void mt_usb_connect(void);
extern void mt_usb_disconnect(void);
extern bool musb_is_host(void);
extern void kernel_power_off(void);
//extern void mt_power_off(void);
//extern int mtkts_bts_get_hw_temp(void);
//extern int get_pwrap_irq_cnt(void);
extern void charger_connect_judge(char on_or_off);
#if 1
extern void Charger_Detect_Init(void);
extern void Charger_Detect_Release(void);
#else
extern void dpdm_hiz_init(void);
extern void dpdm_hiz_release(void);
#endif



/************************************************************
 *
 *   [package function]
 *
 ***********************************************************/
extern unsigned int get_boot_mode(void);

extern bool get_atboot(void);
extern bool get_atfirst(void);
extern bool get_atelsp1(void);
extern bool get_bsp_test_mode(void);
extern bool get_bsptest_battery(void);
extern bool get_normal_mode(void);
extern bool get_charging_mode(void);
extern bool get_meta_mode(void);
extern void charge_plug_notify(bool plugin);
extern void platform_usb_connect(bool connect);
extern void platform_dpdm_hiz(bool hiz);
extern bool platform_get_charge_detect(void);
extern int platform_get_charge_voltage(void);
extern int platform_get_charge_type(void);
extern bool platform_otg_host_real(void);
extern void platform_power_off(void);
extern void platform_register_interrupt_callback(void (EINT_FUNC_PTR) (void));



/************************************************************
 *
 *   [export function]
 *
 ***********************************************************/
extern void psl_usb_plug(void);
extern void wake_up_bat(void);
extern void config_otg_mode(bool enable);
extern bool is_power_path_supported(void);
extern int battery_get_capacity(void);
extern int battery_get_ibat(void);
extern int battery_get_vbat(void);
extern int battery_get_tbat(void);
extern int battery_get_vchg(void);
extern bool upmu_is_chr_det(void);



#endif/* #ifndef __INTERACT_H */
