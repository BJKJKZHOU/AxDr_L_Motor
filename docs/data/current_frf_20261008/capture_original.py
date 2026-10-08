import sys,time,json,struct,subprocess,re,traceback,math
from pathlib import Path
from types import SimpleNamespace
import numpy as np
ROOT=Path('/home/zhouheng/GitHub_Pro/AxDr_L_Motor')
sys.path.insert(0,str(ROOT/'tools'))
import identification_test as b
from ident_swd_trace import openocd_connect
from current_frf import analyze
sys.path.insert(0,'/tmp/trajectory_hw_20261007')
from flash import persistent,state,PORT
OUT=Path('/tmp/current_frf_hw_20261008')
FREQ=[20,40,70,100,150,220,330,500,700,1000,1250,1500,1750,2000,2250,2500,2750,3000,3500,4000,4500,5000]
FRF=['SIGNAL_OUT','SIGNAL_ID_REF','RUN_ID','SIGNAL_ID_PI_OUT','RUN_IQ','RUN_UD']
NOISE=['ADC_IA','ADC_IB','RUN_ID','RUN_IQ','RUN_UD','RUN_UQ']
NORMAL=['ADC_VBUS','RUN_WM','REF_IQ','RUN_THETA_E','RUN_POSITION']
R={'events':[],'bode':[],'noise':[],'timing':{},'fs':20000,'scale':.001,'frf_channels':FRF,'noise_channels':NOISE,'normal_channels':NORMAL}
s=c=t=p=log=None;old={}
expr={'deadline':'Fast_Time.Deadline_Miss','enc_miss':'Fast_Time.Enc_Miss','enc_err':'Encoder.Err_Cnt','crc_err':'MT6835_State.CRC_Err','adc_max':'Fast_Time.ADC_Run_Max','fast_max':'Fast_Time.Fast_Max','fast_drop':'Plot_Fast_Drop','normal_drop':'Plot_Normal_Drop','timing':'Fast_Time.ADC_Cyc','sample_cnt':'Signal_Injection.State.Sample_Cnt','sample_max':'Signal_Injection.State.Sample_Max'}
cmd=['arm-none-eabi-gdb','-q','-batch',str(ROOT/'build/Release/AxDr_L_Motor.elf')]
for k,v in expr.items():cmd+=['-ex',f'printf "ADDR {k} %lu\\n", (unsigned long)&{v}']
addr={k:int(v) for k,v in re.findall(r'ADDR (\w+) (\d+)',subprocess.check_output(cmd,text=True))};assert len(addr)==len(expr)
class Client(b.IdentificationClient):
 def __init__(self,ser):
  super().__init__(ser,SimpleNamespace(timeout=.5));self.names=FRF;self.expected=0;self.reset()
 def reset(self):
  self.raw=bytearray();self.normal=[];self.n=0;self.fs=self.ns=None;self.flost=self.nlost=0;self.guard=None
  self.epoch=self.last_f=self.last_n=time.monotonic();self.next_health=0
 def rd(self,n,tp=2):return self.parameter_read(getattr(b,'PARAM_'+n),tp)
 def wr(self,n,v,tp=2):return self.parameter_write(getattr(b,'PARAM_'+n),tp,v)
 def act(self,n):return self.parameter_action(getattr(b,'ACTION_'+n))
 def process_fast(self,p):
  if len(p)<4 or p[2]!=71:return
  seq,_,n=struct.unpack_from('<HBB',p);p=b.canfd_payload(p,4+n*12)
  if p is None or not n:self.guard='bad FAST';return
  if self.fs is not None:self.flost+=(seq-self.fs-1)&65535
  self.fs=seq;self.last_f=time.monotonic();self.raw.extend(p[4:]);self.n+=n
  x=np.frombuffer(p,dtype='<i2',offset=4).reshape(-1,6)*.001
  if self.names==FRF:
   if np.max(np.hypot(x[:,2],x[:,4]))>2.5:self.guard='dq current >2.5 A'
   if np.max(abs(x[:,5]))>8:self.guard='Ud >8 V'
  elif np.max(np.maximum.reduce([abs(x[:,0]),abs(x[:,1]),abs(x[:,0]+x[:,1])]))>2.5:self.guard='phase current >2.5 A'
 def process_normal(self,p):
  if len(p)<4 or p[2]!=72:return
  p=b.canfd_payload(p,24)
  if p is None or p[3]!=5:self.guard='bad NORMAL';return
  seq=struct.unpack_from('<H',p)[0]
  if self.ns is not None:self.nlost+=(seq-self.ns-1)&65535
  self.ns=seq;self.last_n=time.monotonic();v=struct.unpack_from('<5f',p,4);self.normal.append((self.last_n-self.epoch,seq,*v))
  if not all(math.isfinite(x) for x in v) or not 22<v[0]<28 or abs(v[1])>8 or abs(v[2])>1e-6:self.guard='Vbus/speed/IqRef guard'
 def health(self):
  z=state(self)
  assert z['state']==self.expected and z['ready']==1 and z['valid']==1 and not any(z[k] for k in ['error','trip','fault']),z
 def pump(self,sec):
  end=time.monotonic()+sec
  while time.monotonic()<end:
   self.process(self.parser.feed(self.ser.read(4096)));now=time.monotonic()
   if self.guard:raise RuntimeError(self.guard)
   if self.flost or self.nlost:raise RuntimeError('telemetry sequence gap')
   if now-min(self.last_f,self.last_n)>.3:raise RuntimeError('telemetry absent >300ms')
   if now>self.next_health:self.next_health=now+.25;self.health()
 def start_plot(self,names):
  self.request(b.MSG_PLOT,b.PLOT_STOP,bytes([3]));self.names=names;self.reset()
  for group,cid,ch in [(0,71,names),(1,72,NORMAL)]:self.request(b.MSG_PLOT,b.PLOT_CONFIG,bytes([group,cid,len(ch)])+b''.join(struct.pack('<H',getattr(b,'PARAM_'+n)) for n in ch))
  self.last_f=self.last_n=time.monotonic();self.request(b.MSG_PLOT,b.PLOT_START,bytes([3]))
 def stop_plot(self):self.request(b.MSG_PLOT,b.PLOT_STOP,bytes([3]))
def save(): (OUT/'result.json').write_text(json.dumps(R,indent=2))
def read(a):return t.words(a,1)[0]
def metrics():return {k:read(v) for k,v in addr.items() if k not in ['timing','sample_cnt','sample_max']}
def sig():
 return {'active':c.rd('SIGNAL_ACTIVE',0),'out':c.rd('SIGNAL_OUT'),'id_ref':c.rd('SIGNAL_ID_REF'),'state':c.rd('MOTOR_STATE',0),'sample_cnt':read(addr['sample_cnt']),'sample_max':read(addr['sample_max']),'bdtr':read(0x40012c44)}
def config(f,a,d):
 c.wr('SIGNAL_FREQ_HZ',f);c.wr('SIGNAL_AMP_A',a);c.wr('SIGNAL_TIME_S',d)
def enable():
 c.act('MOTOR_ENABLE');c.expected=1;c.health();assert read(0x40012c44)&32768

def record(tag):
 c.stop_plot();path=OUT/tag;path.mkdir(exist_ok=False)
 (path/'fast.raw').write_bytes(c.raw);np.savetxt(path/'normal.csv',np.array(c.normal),delimiter=',',header='host_t,seq,'+','.join(NORMAL))
 z={'tag':tag,'samples':c.n,'channels':c.names,'fast_lost':c.flost,'normal_lost':c.nlost,'signal':sig(),'metrics':metrics()}
 (path/'capture.json').write_text(json.dumps(z,indent=2));return z

def event(kind,duration=.3):
 config(100,.1,duration if kind=='natural' else 2)
 c.start_plot(FRF);c.pump(.05);pre=c.n;start_t=time.monotonic();c.act('SIGNAL_START');assert c.rd('SIGNAL_ACTIVE',0)==1
 if kind=='natural':c.pump(duration+.1)
 else:
  c.pump(.15);before=sig();assert before['active']==1
  c.act({'Stop':'MOTOR_STOP','Disable':'MOTOR_DISABLE','Abort':'SIGNAL_ABORT'}[kind])
  if kind=='Disable':c.expected=0
  c.pump(.1)
 after=sig();assert after['active']==0 and after['out']==0
 if kind!='Disable':assert after['id_ref']==0 and after['state']==1 and after['bdtr']&32768
 else:assert after['state']==0 and not after['bdtr']&32768
 if kind=='natural':assert after['sample_cnt']==after['sample_max']==round(duration*20000)
 else:assert after['sample_cnt']<after['sample_max']
 z=record('lifecycle_'+kind);z.update({'kind':kind,'before_start_index':pre,'action_host_start':start_t,'after':after})
 x=np.frombuffer(c.raw,dtype='<i2').reshape(-1,6);assert np.max(abs(x[:,0]))>=95
 assert np.all(x[-1000:,0]==0)
 if kind!='Disable':assert np.all(x[-1000:,1]==0)
 R['events'].append(z);save();print('LIFECYCLE '+kind+' '+json.dumps(after),flush=True)
 if kind=='Disable':enable()

def timing(tag,active):
 config(100,.1,5);c.start_plot(FRF)
 if active:c.act('SIGNAL_START')
 c.pump(.1);rows=[]
 for _ in range(100):
  c.pump(.006);rows.append(t.words(addr['timing'],7))
 st=sig();assert st['active']==int(active)
 c.act('SIGNAL_ABORT');c.pump(.1);record('timing_'+tag)
 a=np.array(rows)/160
 R['timing'][tag]={'samples_nonatomic':rows,'summary_us':{name:{'median':float(np.median(a[:,i])),'p95':float(np.percentile(a[:,i],95)),'max_sampled':float(a[:,i].max())} for name,i in [('ADC_trigger_delay',0),('ADC_Run',1),('ADC_ISR',3),('Fast_chain',5)]},'metrics_after':metrics()};save()
 print('TIMING '+tag+' '+json.dumps(R['timing'][tag]['summary_us']),flush=True)
try:
 assert '--run' in sys.argv
 t,p,log=openocd_connect(ROOT/'openocd.cfg',OUT/'swd.json');t.command('poll');t.ensure_running()
 s=b.serial.Serial(PORT,115200,timeout=.003,write_timeout=.5,exclusive=True);c=Client(s);c.health()
 assert not read(0x40012c44)&32768 and c.rd('SIGNAL_ACTIVE',0)==0
 R['adc_before']={hex(a):read(a) for a in [0x50000010,0x50000110,0x50000014,0x50000114,0x40012c40,0x40012c48]}
 assert read(0x50000010)==read(0x50000110)==0x22 and (read(0x50000014)>>9)&7==4 and (read(0x50000114)>>6)&7==4
 R['params_before']=persistent(c);R['metrics_before']=metrics()
 for n,tp in [('MOTOR_MODE',0),('TARGET_TORQUE',2),('TARGET_SPEED',2),('LIMIT_I_MAX',2),('CTRL_CURRENT_SOURCE',0),('CTRL_CURRENT_BW_HZ',2),('CTRL_CURRENT_FF_ENABLE',0),('SIGNAL_FREQ_HZ',2),('SIGNAL_AMP_A',2),('SIGNAL_TIME_S',2)]:old[n]=(tp,c.rd(n,tp))
 R['original']=old;save()
 c.request(b.MSG_PLOT,b.PLOT_STOP,bytes([3]));c.wr('MOTOR_MODE',0,0);c.wr('TARGET_TORQUE',0);c.wr('TARGET_SPEED',0);c.wr('LIMIT_I_MAX',2);c.wr('CTRL_CURRENT_SOURCE',0,0);c.wr('CTRL_CURRENT_BW_HZ',2000);c.wr('CTRL_CURRENT_FF_ENABLE',0,0)
 try:c.act('SIGNAL_START')
 except RuntimeError as e:R['disabled_start_rejection']=str(e)
 else:raise AssertionError('Disabled Start unexpectedly accepted')
 assert c.rd('SIGNAL_ACTIVE',0)==0
 enable()
 for k in ['natural','Stop','Disable','Abort']:event(k)
 timing('off_before',False);timing('on',True);timing('off_after',False)
 for bw in [2000,1000,500]:
  c.act('MOTOR_DISABLE');c.expected=0;c.wr('CTRL_CURRENT_BW_HZ',bw);enable()
  assert c.rd('CTRL_CURRENT_FF_ENABLE',0)==0
  c.start_plot(NOISE);c.pump(1);first=c.n;c.pump(10);last=c.n;z=record(f'noise_{bw}')
  z.update({'bandwidth_hz':bw,'start':first,'end':last});x=np.frombuffer(c.raw,dtype='<i2').reshape(-1,6)[first:last]*.001
  z['std']=x.std(axis=0).tolist();z['Udq_ac_rms_V']=float(np.sqrt(np.var(x[:,4:6],axis=0).sum()));R['noise'].append(z);save()
  print('NOISE '+str(bw)+' '+json.dumps({'std':z['std'],'Udq':z['Udq_ac_rms_V']}),flush=True)
  for f in FREQ:
   config(f,.3,1.0);c.start_plot(FRF);c.pump(.03);c.act('SIGNAL_START');c.pump(1.12)
   assert c.rd('SIGNAL_ACTIVE',0)==0 and c.rd('SIGNAL_OUT')==0
   z=record(f'frf_{bw}_{f}');assert z['signal']['sample_cnt']==z['signal']['sample_max']==20000
   x=np.frombuffer(c.raw,dtype='<i2').reshape(-1,6)
   _,fit=analyze(x[:,:4].copy().tobytes(),f,.3,max(4,f*.05))
   z.update(fit);z['bandwidth_hz']=bw;R['bode'].append(z);save()
   g=fit['closed_loop_Id_over_IdRef'];print(f'FRF BW={bw} f={f} gain={g["gain_db"]:+.3f} dB phase={g["phase_deg"]:+.2f} deg',flush=True)
   for k in ['deadline','enc_miss','enc_err','crc_err','fast_drop','normal_drop']:assert z['metrics'][k]==R['metrics_before'][k],k
 R['completed']=True
except BaseException as e:
 R['error']=repr(e);traceback.print_exc()
 if c and c.raw:(OUT/'interrupted.raw').write_bytes(c.raw)
finally:
 if c:
  try:
   c.act('SIGNAL_ABORT');c.act('MOTOR_DISABLE');c.expected=0;c.stop_plot()
   for n,(tp,v) in old.items():c.wr(n,v,tp)
   R['restored']=all(c.rd(n,tp)==v for n,(tp,v) in old.items());assert R['restored']
   R['params_after']=persistent(c);assert R['params_after']==R['params_before']
   R['adc_after']={k:read(int(k,16)) for k in R['adc_before']};assert R['adc_after']==R['adc_before']
   R['final_state']=state(c);c.health();R['final_signal']=sig();assert not R['final_signal']['bdtr']&32768 and R['final_signal']['active']==0 and R['final_signal']['out']==0
   R['metrics_after']=metrics()
  except BaseException as e:R['cleanup_error']=repr(e);traceback.print_exc()
 save()
 if s:s.close()
 if t:
  if p:t.command('shutdown')
  t.close()
 if p:p.wait(timeout=5)
 if log:log.close()
 print('FINAL '+json.dumps({k:R.get(k) for k in ['completed','error','cleanup_error','restored','final_state','final_signal','metrics_after']}),flush=True)
if not R.get('completed') or R.get('cleanup_error'):raise SystemExit(1)
