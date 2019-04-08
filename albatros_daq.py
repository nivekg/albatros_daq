#!/usr/bin/python
import casperfpga
import casperfpga.snapadc
import argparse
import logging
import time
import numpy
import datetime
import struct
import scio
import os

def initialise_snap(args, adc_tries, fft_of_tries, logger):
	logger.info("Beginning SNAP Board initialisation")
	logger.info("Connecting to SNAP Board at %s:%s"%(args.ip,args.port))
	snap = casperfpga.CasperFpga(args.ip)
	if snap.is_connected():
		logger.info("Connected")
	else:
		logger.error("Failed to connect. Exiting!!!")
		exit(0)
	try:
		logger.info("Programming FPGA")
		snap.upload_to_ram_and_program(args.firmware)
	except e:
		logger.error("Failed to program with error: "+e)
		logger.error("Exiting!!!")
		exit(0)
	else:
		logger.info("FPGA programmed successfully")
	logger.info("Attempting to initialize ADCs")
	snap_adc = casperfpga.snapadc.SNAPADC(snap, ref=None)
	for i in range(adc_tries):
		if snap_adc.init(samplingRate=250, numChannel=4, resolution=8) == 0:
			for j in range(3):
				snap_adc.selectADC(j)
				snap_adc.adc.selectInput([1,2,3,4])
			logger.info("ADC calibration done after %d attempts"%(i+1))
			break
		elif i==adc_tries-1:
			logger.error("ADC calibration failed after %d attempts. Exiting!!!"%(i+1))
			exit(0)
		else:
			logger.error("ADC calibration failed. Retrying.")
	logger.info("Board clock: %f"%snap.estimate_fpga_clock())
	logger.info("Setting fft_shift")
	snap.registers.pfb0_fft_shift.write_int(args.fftshift & 0xffff)
	snap.registers.pfb1_fft_shift.write_int(args.fftshift & 0xffff)
	logger.info("Setting acc_len")
	snap.registers.acc_len.write_int(args.acclen)
	logger.info("Syncing from software trigger")
	snap.registers.cnt_rst.write_int(0)
	snap.registers.sw_sync.write_int(0)
	snap.registers.sw_sync.write_int(1)
	snap.registers.sw_sync.write_int(0)
	snap.registers.cnt_rst.write_int(1)
	snap.registers.cnt_rst.write_int(0)
	time.sleep(2.5)
	pfb0_fft_of = False
	pfb1_fft_of = False
	for i in range(fft_of_tries):
		pfb0_fft_of = pfb0_fft_of or bool(snap.registers.pfb0_fft_of.read_int())
		pfb1_fft_of = pfb1_fft_of or bool(snap.registers.pfb1_fft_of.read_int())
	if pfb0_fft_of:
		logger.warning("pfb0 FFT overflow")
	if pfb1_fft_of:
		logger.warning("pfb1 FFT overflow")
	if not(pfb0_fft_of) and not(pfb1_fft_of):
		logger.info("No FFT overflow detected")
	logger.info("Initialization complete")
	return snap

def read_pols(snap, pols, struct_format):
	pols_dict = {}
	for pol in pols:
		pols_dict[pol] = numpy.array(struct.unpack(struct_format, snap.sbrams[pol].read_raw()), dtype="int64")
		#pols_dict[pol] = numpy.array(struct.unpack(struct_format, snap.read(pol, 2048*8)), dtype="int64")
	return pols_dict

def read_registers(snap, regs):
	reg_dict = {}
	for r in regs:
		reg_dict[r] = numpy.array(snap.registers[r].read_int())
	return reg_dict

def get_fpga_temperature(snap):
	TEMP_OFFSET = 0x0
	x = snap.read_int("xadc", TEMP_OFFSET)
	return (x >> 4)*503.975/4096.00-273.15

def get_rpi_temperature():
	x = open("/sys/class/thermal/thermal_zone0/temp", "r")
	temp = numpy.int32(x.readline())/1000
	x.close()
	return temp

def acquire_data(snap, args, pols, regs, time_frag_length, logger):
	while True:
		start_time = time.time()
		if start_time>1e5:
			time_frag = str(start_time)[:time_frag_length]
		else:
			logger.warning("Start time in acquire data seems to be near zero")
		outsubdir = "%s/%s/%s"%(args.outdir, time_frag, str(numpy.int64(start_time)))
		os.makedirs(outsubdir)
		logger.info("Writing current data to %s"%outsubdir)
		start_raw_files = {}
		end_raw_files = {}
		scio_files = {}
		file_sys_timestamp1 = open("%s/time_sys_start.raw"%outsubdir, "w")
		file_sys_timestamp2 = open("%s/time_sys_stop.raw"%outsubdir, "w")
		file_fpga_temp = open("%s/fpga_temp.raw"%outsubdir, "w")
		file_pi_temp = open("%s/pi_temp.raw"%outsubdir, "w")
		for reg in regs:
			start_raw_files[reg] = open("%s/%s1.raw"%(outsubdir, reg), "w")
			end_raw_files[reg] = open("%s/%s2.raw"%(outsubdir, reg), "w")
		for pol in pols:
			scio_files[pol] = scio.scio("%s/%s.scio"%(outsubdir, pol), compress=args.compress)
		acc_cnt = 0 #set higher to not read startup fft data ~3 acc_cnt
		while time.time()-start_time < args.tfile*60:
			new_acc_cnt = read_registers(snap, ["acc_cnt"]) 
			if new_acc_cnt > acc_cnt:
				print(new_acc_cnt)
				print(time.ctime())
				acc_cnt = new_acc_cnt
				start_sys_timestamp = time.time()
				start_reg_data = read_registers(snap, regs)
				pol_data = read_pols(snap, pols, ">2048q")
				end_reg_data = read_registers(snap, regs)
				end_sys_timestamp = time.time()
				read_time = end_sys_timestamp-start_sys_timestamp
				print("Read took: "+str(read_time))
				if start_reg_data["acc_cnt"] != end_reg_data["acc_cnt"]:
					logger.warning("Accumulation length changed during read")
				for reg in regs:
					numpy.array(start_reg_data[reg]).tofile(start_raw_files[reg])
					start_raw_files[reg].flush()
					numpy.array(end_reg_data[reg]).tofile(end_raw_files[reg])
					end_raw_files[reg].flush()
				for pol in pols:
					scio_files[pol].append(pol_data[pol])
				numpy.array(start_sys_timestamp).tofile(file_sys_timestamp1)
				numpy.array(get_fpga_temperature(snap)).tofile(file_fpga_temp)
				numpy.array(get_rpi_temperature()).tofile(file_pi_temp)
				numpy.array(end_sys_timestamp).tofile(file_sys_timestamp2)
				file_sys_timestamp1.flush()
				file_fpga_temp.flush()
				file_pi_temp.flush()
				file_sys_timestamp2.flush()
		for pol in pols:
			scio_files[pol].close()
		for reg in regs:
			start_raw_files[reg].close()
			end_raw_files[reg].close()
		file_sys_timestamp1.close()
		file_fpga_temp.close()
		file_pi_temp.close()
		file_sys_timestamp2.close()
	
if __name__ == "__main__":
	#Script arguments
	parser = argparse.ArgumentParser()
	parser.add_argument("ip", help = "SNAP board hostname or ip address")
	parser.add_argument("-p", "--port", help="SNAP board port")
	parser.add_argument("-f", "--firmware", help="Firmware file to program")
	parser.add_argument("-s", "--fftshift", type=lambda x: int(x,0), help="Sets FFT shift for pfb0 and pfb1")
	parser.add_argument("-a", "--acclen", type=int, help="Sets accumulation length")
	parser.add_argument("-l", "--logdir", help="Directory for storing log files")
	parser.add_argument("-o", "--outdir", help="Directory for storing data files")
	parser.add_argument("-z", "--compress", default='', help="Command to use to compress data files, if desired")
	parser.add_argument("-t", "--tfile", type=int, default=15, help="Number of minutes of data in each file subdirectory [default: %default]")
	args = parser.parse_args()
	#Setup logging
	logger = logging.getLogger("albatros_daq")
	logger.setLevel(logging.DEBUG)
	file_logger = logging.FileHandler(args.logdir+"/albatros_"+datetime.datetime.now().strftime("%d%m%Y_%H%M%S")+".log")
	file_format = logging.Formatter("%(asctime)s %(name)-12s %(message)s", "%d-%m-%Y %H:%M:%S")
	file_logger.setFormatter(file_format)
	file_logger.setLevel(logging.DEBUG)
	logger.addHandler(file_logger)
	#Log script arguments
	logger.info("SNAP board Hostname/ip address: %s"%args.ip)
	logger.info("SNAP board Port: %s"%args.port)
	logger.info("Firmware file: %s"%args.firmware)
	logger.info("FFT shift: %d"%args.fftshift)
	logger.info("Accumulation length: %d"%args.acclen)
	logger.info("Log directory: %s"%args.logdir)
	logger.info("Out directory: %s"%args.outdir)
	logger.info("Minutes per file: %d"%args.tfile)
	logger.info("Compress format: %s"%args.compress)
	pols = []
	for i in range(4):
		for j in range(i, 4):
			if i==j:
				pols.append("pol%d%d"%(i, i))
			else:
				pols.append("pol%d%dr"%(i, j))
				pols.append("pol%d%di"%(i, j))
	regs = ["sync_cnt", "pfb1_fft_of", "acc_cnt", "sys_clkcounter", "pfb0_fft_of"]
	try:
		snap = initialise_snap(args, 3, 5, logger)
		acquire_data(snap, args, pols, regs, 5, logger)
	finally:
		logger.info("Terminating DAQ at %s"%datetime.datetime.now().strftime("%d-%m-%Y %H:%M:%S"))
