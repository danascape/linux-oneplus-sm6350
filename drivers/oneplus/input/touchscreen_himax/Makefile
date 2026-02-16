#
# Makefile for the touchscreen drivers.
#
#obj-y += samsung/
ccflags-y := -I$(srctree)/include/drm -I$(srctree)/techpack/display/msm -I$(srctree)/techpack/display/msm/dsi
ccflags-y += -I$(srctree)/techpack/display/msm/sde
subdir-ccflags-y += -I$(srctree)/techpack/display/msm -I$(srctree)/techpack/display/msm/dsi

obj-y += util_interface/
obj-y += touchpanel_common_driver_himax.o
obj-y += update_tpfw_notifier.o
obj-y += himax/
#obj-y += synaptics/
