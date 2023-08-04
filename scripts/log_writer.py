#------------------------------------------------------------------------------------------------------------
# File: scripts/log_writer.py
# Description: This file contains the dictionary of commands and resource types used in the system.
# Author: Balachandra Bhat (github.com/bnbhat)
# Date: 2023-08-03
# Version: 1.0
#------------------------------------------------------------------------------------------------------------

import os
import time
import datetime
import xlsxwriter
import command_dict

class LogWriter:

	def __init__(self, file_name):
		try:
			self.base_dir = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir))
			self.excel_file_path = os.path.join(self.base_dir, 'logs', f"{file_name}_{datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.xlsx")
			self.backup_syslog_file_path = os.path.join(self.base_dir, '.system_logs', f"{file_name}_{datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.txt")
		except Exception as e:
			print(f"Error creating log file: {e}")
			return
		try:
			self.workbook = xlsxwriter.Workbook(self.excel_file_path)
			self.log_sheet = self.workbook.add_worksheet()
			self.log_sheet.write('A1', 'UUID')
			self.log_sheet.write('B1', 'Command')
			self.log_sheet.write('C1', 'Resource Type')
			self.log_sheet.write('D1', 'Message')
			self.log_sheet.write('E1', 'Time')
		except Exception as e:
			print(f"Error creating excel file: {e}")
			return
		self.index = 2

	def log(self, uuid, command_int, resource_type_int, msg, time=None):
		if time is None:
			time = datetime.datetime.now().strftime("%H:%M:%S") 

		command = command_dict.get_status_code(command_int)
		resource_type = command_dict.get_resource_type(resource_type_int)
		
		self.backup_syslog(uuid, command, resource_type, msg, time) #backup log to txt file
		self.log_excel(uuid, command, resource_type, msg, time) #log to excel file
	
	def log_excel(self, uuid, command, resource_type, msg, time):
		try:
			self.log_sheet.write(f'A{self.index}', uuid)    
			self.log_sheet.write(f'B{self.index}', command)  
			self.log_sheet.write(f'C{self.index}', resource_type) 
			self.log_sheet.write(f'D{self.index}', msg)    
			self.log_sheet.write(f'E{self.index}', time) 
			self.index += 1
		except Exception as e:
			print(f"Error writing to excel file: {e}")

	def backup_syslog(self, uuid, command, resource_type, msg, time):
		with open(self.backup_syslog_file_path, 'a') as txt_file:  
			try:
				log_line = f"{uuid}, {command}, {resource_type}, {msg}, {time}\n"
				txt_file.write(log_line)
			except Exception as e:
				print(f"Error writing to syslog file: {e}")

	def close(self):
		try:
			self.workbook.close()
		except Exception as e:
			print(f"Error closing excel file: {e}")

# Test
if __name__ == "__main__":

	print("Starting log writer - test")

	logger = LogWriter('log_file')
	for i in range(1, 20, 2):
		print(f"log entry : {i}")
		logger.log("11111", 3, 1, f"log entry : {i}")
		time.sleep(1)
		print(f"log entry : {i+1}")
		logger.log("11112", 1, 1, f"log entry : {i+1}")
		time.sleep(1)
	logger.close()