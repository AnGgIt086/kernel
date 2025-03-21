obj-y += vivo-codec-common.o
obj-y += vivo_soc_codec.o
ifeq ($(CONFIG_SND_SOC_MTK_SMART_PHONE),y)
obj-y += vivo_smartpa_cali.o
endif
#obj-y += ak4376.o
#obj-y += tas256x/
obj-y += tfa9874/
obj-y += tfa9894/
obj-y += aw881xx/
obj-y += typec/
##obj-y += aw87xxx/
obj-y += aw87390/
obj-y += aw88263/
obj-y += aw882xx/
#vivo linzhinan add for tas2562
obj-y += tas2562/
obj-y += fs1599/
#add by yanghen for sipa 
obj-$(CONFIG_VIVO_CODEC_SIA81XX) += sipa/
# vivo audio GuoYijia add for PD2279 SIA8159 to distinguish begin
obj-$(CONFIG_SND_SOC_SIAXXXX) += sipa_i2c/
# vivo audio GuoYijia add for PD2279 SIA8159 to distinguish end
obj-$(CONFIG_VIVO_REGDUMP) += vivo-regdump.o
#add by yanghen for fsm1519n 
obj-$(CONFIG_VIVO_CODEC_FS1519N) += fs1519n/
