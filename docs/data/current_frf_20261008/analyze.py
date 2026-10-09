import json,sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[3];sys.path.insert(0,str(ROOT/'tools'))
from current_frf import sinusoid_fit
P=Path(__file__).resolve().parent;r=json.loads((P/'result.json').read_text());fs=20000
out={'bode':[],'bandwidth':{},'noise':[],'lifecycle':[],'timing':r['timing']}
colors={500:'#1f77b4',1000:'#ff7f0e',2000:'#d62728'}
fig,axs=plt.subplots(2,2,figsize=(12,8),sharex='col')
for bw in [500,1000,2000]:
 pts=[e for e in r['bode'] if e['bandwidth_hz']==bw]
 if not pts:continue
 ff=[];gg=[];ph=[];ug=[];up=[]
 for e in pts:
  x=np.fromfile(P/e['tag']/'fast.raw',dtype='<i2').reshape(-1,6)*.001
  f=e['frequency_hz'];nz=np.flatnonzero(abs(x[:,0])>.006);start=nz[0]+round(max(4,f*.05)*fs/f);end=nz[-1]+1;x=x[start:end]
  refs=[];curs=[];gains=[];phases=[]
  for a in np.array_split(x,8):
   tt=np.arange(len(a))/fs;ref,_=sinusoid_fit(a[:,1],f,tt);cur,_=sinusoid_fit(a[:,2],f,tt)
   refs.append(ref);curs.append(cur);gains.append(20*np.log10(abs(cur/ref)));phases.append(np.angle(cur/ref))
  refs=np.array(refs);curs=np.array(curs);h=e['closed_loop_Id_over_IdRef'];u=e['controller_UdPI_over_IdRef']
  z={'bandwidth_setting_hz':bw,'frequency_hz':f,**h,'ref_amplitude_A':e['input_amplitude_measured_a'],'fit_residual':e['fit_residual_rms'],'gain_segment_std_db':float(np.std(gains,ddof=1)),'phase_segment_std_deg':float(np.std(np.unwrap(phases),ddof=1)*180/np.pi),'segment_coherence':float(abs(np.mean(curs*refs.conj()))**2/(np.mean(abs(curs)**2)*np.mean(abs(refs)**2))),'Ud_vs_UdPI_max_diff_V':float(abs(x[:,5]-x[:,3]).max())}
  out['bode'].append(z);ff.append(f);gg.append(h['gain_db']);ph.append(h['phase_deg']);ug.append(u['gain_db']);up.append(u['phase_deg'])
 ff=np.array(ff);gg=np.array(gg);ph=np.rad2deg(np.unwrap(np.deg2rad(ph)))
 for ax,val in [(axs[0,0],gg),(axs[1,0],ph),(axs[0,1],ug),(axs[1,1],np.rad2deg(np.unwrap(np.deg2rad(up))))]:ax.semilogx(ff,val,'o-',ms=3,color=colors[bw],label=f'PI {bw} Hz')
 crossing=[]
 for i in range(1,len(ff)):
  if gg[i-1]>-3>=gg[i]:crossing.append(float(np.exp(np.interp(-3,[gg[i],gg[i-1]],[np.log(ff[i]),np.log(ff[i-1])]))))
 out['bandwidth'][str(bw)]={'minus3db_hz_log_interpolated':crossing,'max_measured_gain_db':float(gg.max()),'max_measured_gain_freq_hz':float(ff[gg.argmax()]),'last_measured_hz':float(ff[-1]),'last_gain_db':float(gg[-1])}
for ax in axs.flat:ax.grid(True,which='both',alpha=.3);ax.legend()
axs[0,0].axhline(-3,ls='--',c='gray');axs[0,0].set_ylabel('Id / IdRef (dB)');axs[1,0].set_ylabel('Phase (deg)');axs[0,1].set_ylabel('UdPI / IdRef (dB V/A)');axs[1,1].set_ylabel('Phase (deg)')
axs[1,0].set_xlabel('Frequency (Hz)');axs[1,1].set_xlabel('Frequency (Hz)')
fig.suptitle('Measured d-axis sine FRF | 0.3 A peak | 2x47.5 cycles | FF off')
fig.tight_layout();fig.savefig(P/'bode.png',dpi=180);fig.savefig(P/'bode.svg');plt.close(fig)
fig,ax=plt.subplots(2,2,figsize=(12,8))
for e in r['noise']:
 bw=e['bandwidth_hz'];x=np.fromfile(P/e['tag']/'fast.raw',dtype='<i2').reshape(-1,6)[e['start']:e['end']]*.001
 nfft=8192;win=np.hanning(nfft);segs=np.array([x[j:j+nfft] for j in range(0,len(x)-nfft+1,nfft//2)]);segs-=segs.mean(axis=1,keepdims=True)
 psd=np.mean(abs(np.fft.rfft(segs*win[None,:,None],axis=1))**2,axis=0)/(fs*np.sum(win**2));psd[1:-1]*=2;freq=np.fft.rfftfreq(nfft,1/fs);df=fs/nfft
 np.savez(P/f'psd_{bw}.npz',frequency_hz=freq,psd=psd,channels=r['noise_channels'])
 z={'bandwidth_setting_hz':bw,'seconds':len(x)/fs,'std':x.std(axis=0).tolist(),'mean':x.mean(axis=0).tolist(),'Udq_ac_rms_V':e['Udq_ac_rms_V'],'psd_method':{'nfft':nfft,'overlap':nfft//2,'segments':len(segs),'window':'Hann','detrend':'segment mean','delta_f_Hz':df},'band_rms':{f'{lo}..{hi}':np.sqrt(psd[(freq>=lo)&(freq<hi)].sum(axis=0)*df).tolist() for lo,hi in [(1,1000),(1000,5000),(5000,10001)]}}
 out['noise'].append(z)
 for a,idx,title,unit in [(ax[0,0],2,'Id feedback','A²/Hz'),(ax[0,1],3,'Iq feedback','A²/Hz'),(ax[1,0],4,'Ud command','V²/Hz'),(ax[1,1],5,'Uq command','V²/Hz')]:a.semilogx(freq[1:],10*np.log10(np.maximum(psd[1:,idx],1e-20)),lw=.8,color=colors[bw],label=f'PI {bw} Hz');a.set_title(title);a.set_ylabel('PSD (dB '+unit+')');a.set_xlabel('Frequency (Hz)');a.set_xlim(10,10000)
for a in ax.flat:a.grid(True,which='both',alpha=.3);a.legend()
fig.suptitle('Zero-reference enabled noise | No injection | Same ADC and PI source')
fig.tight_layout();fig.savefig(P/'noise_psd.png',dpi=180);fig.savefig(P/'noise_psd.svg');plt.close(fig)
for e in r['events']:
 x=np.fromfile(P/e['tag']/'fast.raw',dtype='<i2').reshape(-1,6)*.001;nz=np.flatnonzero(abs(x[:,0])>.001)
 first=nz[0]-1;last=nz[-1]+1
 z={'kind':e['kind'],'injection_equals_IdRef_while_nonzero':bool(np.all(x[nz,0]==x[nz,1])),'first_injection_sample_inferred':int(first),'last_nonzero_plus_one':int(last),'nonzero_envelope_samples':int(last-first),'tail_out':float(x[-1,0]),'tail_IdRef':float(x[-1,1]),'signal':e['after']}
 if e['kind']=='natural':
  sine=.1*np.sin(2*np.pi*100*np.arange(6000)/fs);z['max_sine_error_A']=float(abs(x[first:first+6000,0]-sine).max());z['post_duration_all_zero']=bool(np.all(x[first+6000:,0:2]==0))
 out['lifecycle'].append(z)
(P/'analysis.json').write_text(json.dumps(out,indent=2))
print(json.dumps({'bandwidth':out['bandwidth'],'noise':[{'bw':e['bandwidth_setting_hz'],'std':e['std'],'Udq':e['Udq_ac_rms_V']} for e in out['noise']],'min_coherence':min(e['segment_coherence'] for e in out['bode']),'max_gain_segment_std_db':max(e['gain_segment_std_db'] for e in out['bode']),'max_phase_segment_std_deg':max(e['phase_segment_std_deg'] for e in out['bode']),'max_limit_difference':max(e['Ud_vs_UdPI_max_diff_V'] for e in out['bode']),'lifecycle':out['lifecycle']},indent=2))
