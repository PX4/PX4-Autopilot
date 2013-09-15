# GDB/Python functions for dealing with NuttX

import gdb, gdb.types

class NX_register_set(object):
	"""Copy of the registers for a given context"""

	v7_regmap = {
		'R13':		0,
		'SP':		0,
		'PRIORITY':	1,
		'R4':		2,
		'R5':		3,
		'R6':		4,
		'R7':		5,
		'R8':		6,
		'R9':		7,
		'R10':		8,
		'R11':		9,
		'EXC_RETURN':	10,
		'R0':		11,
		'R1':		12,
		'R2':		13,
		'R3':		14,
		'R12':		15,
		'R14':		16,
		'LR':		16,
		'R15':		17,
		'PC':		17,
		'XPSR':		18,
	}

	v7em_regmap = {
		'R13':		0,
		'SP':		0,
		'PRIORITY':	1,
		'R4':		2,
		'R5':		3,
		'R6':		4,
		'R7':		5,
		'R8':		6,
		'R9':		7,
		'R10':		8,
		'R11':		9,
		'EXC_RETURN':	10,
		'R0':		27,
		'R1':		28,
		'R2':		29,
		'R3':		30,
		'R12':		31,
		'R14':		32,
		'LR':		32,
		'R15':		33,
		'PC':		33,
		'XPSR':		34,
	}

	regs = dict()

	def __init__(self, xcpt_regs):
		if xcpt_regs is None:
			self.regs['R0']         = long(gdb.parse_and_eval('$r0'))
			self.regs['R1']         = long(gdb.parse_and_eval('$r1'))
			self.regs['R2']         = long(gdb.parse_and_eval('$r2'))
			self.regs['R3']         = long(gdb.parse_and_eval('$r3'))
			self.regs['R4']         = long(gdb.parse_and_eval('$r4'))
			self.regs['R5']         = long(gdb.parse_and_eval('$r5'))
			self.regs['R6']         = long(gdb.parse_and_eval('$r6'))
			self.regs['R7']         = long(gdb.parse_and_eval('$r7'))
			self.regs['R8']         = long(gdb.parse_and_eval('$r8'))
			self.regs['R9']         = long(gdb.parse_and_eval('$r9'))
			self.regs['R10']        = long(gdb.parse_and_eval('$r10'))
			self.regs['R11']        = long(gdb.parse_and_eval('$r11'))
			self.regs['R12']        = long(gdb.parse_and_eval('$r12'))
			self.regs['R13']        = long(gdb.parse_and_eval('$r13'))
			self.regs['SP']         = long(gdb.parse_and_eval('$sp'))
			self.regs['R14']        = long(gdb.parse_and_eval('$r14'))
			self.regs['LR']         = long(gdb.parse_and_eval('$lr'))
			self.regs['R15']        = long(gdb.parse_and_eval('$r15'))
			self.regs['PC']         = long(gdb.parse_and_eval('$pc'))
			self.regs['XPSR']       = long(gdb.parse_and_eval('$xpsr'))
		else:
			for key in self.v7em_regmap.keys():
				self.regs[key] = int(xcpt_regs[self.v7em_regmap[key]])


	@classmethod
	def with_xcpt_regs(cls, xcpt_regs):
		return cls(xcpt_regs)

	@classmethod
	def for_current(cls):
		return cls(None)

	def __format__(self, format_spec):
		return format_spec.format(
			registers	 = self.registers
			)

	@property
	def registers(self):
		return self.regs


class NX_task(object):
	"""Reference to a NuttX task and methods for introspecting it"""

	def __init__(self, tcb_ptr):
		self._tcb = tcb_ptr.dereference()
		self._group = self._tcb['group'].dereference()
		self.pid = tcb_ptr['pid']

	@classmethod
	def for_tcb(cls, tcb):
		"""return a task with the given TCB pointer"""
		pidhash_sym = gdb.lookup_global_symbol('g_pidhash')
		pidhash_value = pidhash_sym.value()
		pidhash_type = pidhash_sym.type
		for i in range(pidhash_type.range()[0],pidhash_type.range()[1]):
			pidhash_entry = pidhash_value[i]
			if pidhash_entry['tcb'] == tcb:
				return cls(pidhash_entry['tcb'])
		return None

	@classmethod
	def for_pid(cls, pid):
		"""return a task for the given PID"""
		pidhash_sym = gdb.lookup_global_symbol('g_pidhash')
		pidhash_value = pidhash_sym.value()
		pidhash_type = pidhash_sym.type
		for i in range(pidhash_type.range()[0],pidhash_type.range()[1]):
			pidhash_entry = pidhash_value[i]
			if pidhash_entry['pid'] == pid:
				return cls(pidhash_entry['tcb'])
		return None

	@staticmethod
	def pids():
		"""return a list of all PIDs"""
		pidhash_sym = gdb.lookup_global_symbol('g_pidhash')
		pidhash_value = pidhash_sym.value()
		pidhash_type = pidhash_sym.type
		result = []
		for i in range(pidhash_type.range()[0],pidhash_type.range()[1]):
			entry = pidhash_value[i]
			pid = int(entry['pid'])
			if pid is not -1:
				result.append(pid)
		return result

	@staticmethod
	def tasks():
		"""return a list of all tasks"""
		tasks = []
		for pid in NX_task.pids():
			tasks.append(NX_task.for_pid(pid))
		return tasks

	def _state_is(self, state):
		"""tests the current state of the task against the passed-in state name"""
		statenames = gdb.types.make_enum_dict(gdb.lookup_type('enum tstate_e'))
		if self._tcb['task_state'] == statenames[state]:
			return True
		return False

	@property
	def stack_used(self):
		"""calculate the stack used by the thread"""
		if 'stack_used' not in self.__dict__:
			stack_base = self._tcb['stack_alloc_ptr'].cast(gdb.lookup_type('unsigned char').pointer())
			if stack_base == 0:
				self.__dict__['stack_used'] = 0
			else:
				stack_limit = self._tcb['adj_stack_size']
				for offset in range(0, stack_limit):
					if stack_base[offset] != 0xff:
						break
				self.__dict__['stack_used'] = stack_limit - offset
		return self.__dict__['stack_used']

	@property
	def name(self):
		"""return the task's name"""
		return self._tcb['name'].string()

	@property
	def state(self):
		"""return the name of the task's current state"""
		statenames = gdb.types.make_enum_dict(gdb.lookup_type('enum tstate_e'))
		for name,value in statenames.iteritems():
			if value == self._tcb['task_state']:
				return name
		return 'UNKNOWN'

	@property
	def waiting_for(self):
		"""return a description of what the task is waiting for, if it is waiting"""
		if self._state_is('TSTATE_WAIT_SEM'):
			waitsem = self._tcb['waitsem'].dereference()
			waitsem_holder = waitsem['holder']
			holder = NX_task.for_tcb(waitsem_holder['htcb'])
			if holder is not None:
				return '{}({})'.format(waitsem.address, holder.name)
			else:
				return '{}(<bad holder>)'.format(waitsem.address)
		if self._state_is('TSTATE_WAIT_SIG'):
			return 'signal'
		return None

	@property
	def is_waiting(self):
		"""tests whether the task is waiting for something"""
		if self._state_is('TSTATE_WAIT_SEM') or self._state_is('TSTATE_WAIT_SIG'):
			return True

	@property
	def is_runnable(self):
		"""tests whether the task is runnable"""
		if (self._state_is('TSTATE_TASK_PENDING') or 
			self._state_is('TSTATE_TASK_READYTORUN') or 
			self._state_is('TSTATE_TASK_RUNNING')):
			return True
		return False

	@property
	def file_descriptors(self):
		"""return a dictionary of file descriptors and inode pointers"""
		filelist = self._group['tg_filelist']
		filearray = filelist['fl_files']
		result = dict()
		for i in range(filearray.type.range()[0],filearray.type.range()[1]):
			inode = long(filearray[i]['f_inode'])
			if inode != 0:
				result[i] = inode
		return result

	@property
	def registers(self):
		if 'registers' not in self.__dict__:
			registers = dict()
			if self._state_is('TSTATE_TASK_RUNNING'):
				registers = NX_register_set.for_current().registers
			else:
				context = self._tcb['xcp']
				regs = context['regs']
				registers = NX_register_set.with_xcpt_regs(regs).registers

			self.__dict__['registers'] = registers
		return self.__dict__['registers']

	def __repr__(self):
		return "<NX_task {}>".format(self.pid)

	def __str__(self):
		return "{}:{}".format(self.pid, self.name)

	def __format__(self, format_spec):
		return format_spec.format(
			pid              = self.pid,
			name             = self.name,
			state            = self.state,
			waiting_for      = self.waiting_for,
			stack_used       = self.stack_used,
			stack_limit      = self._tcb['adj_stack_size'],
			file_descriptors = self.file_descriptors,
			registers	 = self.registers
			)

class NX_show_task (gdb.Command):
	"""(NuttX) prints information about a task"""

	def __init__(self):
		super(NX_show_task, self).__init__("show task", gdb.COMMAND_USER)

	def invoke(self, arg, from_tty):
		t = NX_task.for_pid(int(arg))
		if t is not None:
			my_fmt = 'PID:{pid}  name:{name}  state:{state}\n'
			my_fmt += '  stack used {stack_used} of {stack_limit}\n'
			if t.is_waiting:
				my_fmt += '  waiting for {waiting_for}\n'
			my_fmt += '  open files: {file_descriptors}\n'
			my_fmt += '  R0  {registers[R0]:#010x} {registers[R1]:#010x} {registers[R2]:#010x} {registers[R3]:#010x}\n'
			my_fmt += '  R4  {registers[R4]:#010x} {registers[R5]:#010x} {registers[R6]:#010x} {registers[R7]:#010x}\n'
			my_fmt += '  R8  {registers[R8]:#010x} {registers[R9]:#010x} {registers[R10]:#010x} {registers[R11]:#010x}\n'
			my_fmt += '  R12 {registers[PC]:#010x}\n'
			my_fmt += '  SP  {registers[SP]:#010x} LR {registers[LR]:#010x} PC {registers[PC]:#010x} XPSR {registers[XPSR]:#010x}\n'
			print format(t, my_fmt)

class NX_show_tasks (gdb.Command):
	"""(NuttX) prints a list of tasks"""

	def __init__(self):
		super(NX_show_tasks, self).__init__('show tasks', gdb.COMMAND_USER)

	def invoke(self, args, from_tty):
		tasks = NX_task.tasks()
		for t in tasks:
			print format(t, '{pid:<2} {name:<16} {state:<20} {stack_used:>4}/{stack_limit:<4}')

NX_show_task()
NX_show_tasks()

class NX_show_heap (gdb.Command):
	"""(NuttX) prints the heap"""

	def __init__(self):
		super(NX_show_heap, self).__init__('show heap', gdb.COMMAND_USER)
		if gdb.lookup_type('struct mm_allocnode_s').sizeof == 8:
			self._allocflag = 0x80000000
			self._allocnodesize = 8
		else:
			self._allocflag = 0x8000
			self._allocnodesize = 4

	def _node_allocated(self, allocnode):
		if allocnode['preceding'] & self._allocflag:
			return True
		return False

	def _node_size(self, allocnode):
		return allocnode['size'] & ~self._allocflag

	def _print_allocations(self, region_start, region_end):
		if region_start >= region_end:
			raise gdb.GdbError('heap region {} corrupt'.format(hex(region_start)))
		nodecount = region_end - region_start
		print 'heap {} - {}'.format(region_start, region_end)
		cursor = 1
		while cursor < nodecount:
			allocnode = region_start[cursor]
			if self._node_allocated(allocnode):
				state = ''
			else:
				state = '(free)'
			print '  {} {} {}'.format(allocnode.address + 8, self._node_size(allocnode), state)
			cursor += self._node_size(allocnode) / self._allocnodesize

	def invoke(self, args, from_tty):
		heap = gdb.lookup_global_symbol('g_mmheap').value()
		nregions = heap['mm_nregions']
		region_starts = heap['mm_heapstart']
		region_ends = heap['mm_heapend']
		print '{} heap(s)'.format(nregions)
		# walk the heaps
		for i in range(0, nregions):
			self._print_allocations(region_starts[i], region_ends[i])

NX_show_heap()

class NX_show_interrupted_thread (gdb.Command):
	"""(NuttX) prints the register state of an interrupted thread when in interrupt/exception context"""

	def __init__(self):
		super(NX_show_interrupted_thread, self).__init__('show interrupted-thread', gdb.COMMAND_USER)

	def invoke(self, args, from_tty):
		regs = gdb.lookup_global_symbol('current_regs').value()
		if regs is 0:
			raise gdb.GdbError('not in interrupt context')
		else:
			registers = NX_register_set.with_xcpt_regs(regs)
			my_fmt = ''
			my_fmt += '  R0  {registers[R0]:#010x} {registers[R1]:#010x} {registers[R2]:#010x} {registers[R3]:#010x}\n'
			my_fmt += '  R4  {registers[R4]:#010x} {registers[R5]:#010x} {registers[R6]:#010x} {registers[R7]:#010x}\n'
			my_fmt += '  R8  {registers[R8]:#010x} {registers[R9]:#010x} {registers[R10]:#010x} {registers[R11]:#010x}\n'
			my_fmt += '  R12 {registers[PC]:#010x}\n'
			my_fmt += '  SP  {registers[SP]:#010x} LR {registers[LR]:#010x} PC {registers[PC]:#010x} XPSR {registers[XPSR]:#010x}\n'
			print format(registers, my_fmt)

NX_show_interrupted_thread()
