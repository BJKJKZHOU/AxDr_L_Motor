#!/usr/bin/env python3
"""Offline FRF regressions; never opens a hardware connection."""
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

import current_frf as frf


class BodeTest(unittest.TestCase):
    def setUp(self):
        self.f = np.geomspace(.001, 100, 4001)
        self.l = 4 / (1 + 1j * self.f) ** 3
        self.t = self.l / (1 + self.l)

    def metrics(self, **kwargs):
        return frf.bode_metrics(self.f, self.l, self.t, quality="valid", **kwargs)

    def test_known_third_order_loop(self):
        m = self.metrics()
        fc = np.sqrt(4 ** (2 / 3) - 1)
        self.assertAlmostEqual(m['gain_crossover_hz'], fc, places=4)
        self.assertAlmostEqual(m['phase_margin_deg'], 180 - 3*np.rad2deg(np.arctan(fc)), places=3)
        self.assertAlmostEqual(m['phase_crossing_hz'], np.sqrt(3), places=4)
        self.assertAlmostEqual(m['gain_margin_db'], 20*np.log10(2), places=3)

    def test_negative_margins_are_not_wrapped_positive(self):
        l = 12 / (1 + 1j*self.f)**3
        m = frf.bode_metrics(self.f,l,l/(1+l),quality='valid')
        self.assertLess(m['phase_margin_deg'],0)
        self.assertAlmostEqual(m['gain_margin_db'],20*np.log10(8/12),places=3)

    def test_leading_bad_point_does_not_shift_margins(self):
        original = self.metrics()
        self.l[0] = 10 * np.exp(1j*np.deg2rad(175))
        changed = self.metrics()
        self.assertAlmostEqual(changed['phase_margin_deg'], original['phase_margin_deg'])
        # The extra low-frequency negative-real crossing is also considered.
        self.assertAlmostEqual(changed['gain_margin_db'], original['gain_margin_db'])

    def test_unknown_and_invalid_measurements_have_no_metrics(self):
        for quality in ['unknown', 'invalid']:
            m = frf.bode_metrics(self.f, self.l, self.t, quality=quality,
                                 quality_reason='oscillatory' if quality=='invalid' else '')
            for key in ['phase_margin_deg', 'gain_margin_db', 'relative_minus_3db_hz',
                        'gain_crossover_hz', 'phase_crossing_hz']:
                self.assertIsNone(m[key])
            self.assertTrue(m['phase_margin_reason'])
            self.assertEqual(m['relative_minus_3db_all_hz'], [])

    def test_missing_crossing_is_not_infinite_margin(self):
        m = frf.bode_metrics([1, 2, 3], [.1-.1j]*3, [.1]*3, quality='valid')
        self.assertIsNone(m['phase_margin_deg'])
        self.assertIsNone(m['gain_margin_db'])
        self.assertIn('no bracketed', m['phase_margin_reason'])

    def test_exact_crossing_once_and_no_endpoint_or_tangent(self):
        self.assertEqual(len(frf._crossings([1,2,3], [1,0,-1], 0)), 1)
        for values in ([0,-1,-2], [2,1,0], [1,0,1], [1,0,0], [np.nan,0,-1]):
            self.assertEqual(frf._crossings([1,2,3], values, 0), [])

    def test_rising_crossings_are_not_ignored(self):
        hz = np.array([1,1.4,2,2.8,4])
        l = np.array([2,.5,2,.5,.2])*np.exp(1j*np.deg2rad([-90,-90,-170,-170,-170]))
        m = frf.bode_metrics(hz,l,l/(1+l),quality='valid')
        self.assertAlmostEqual(m['phase_margin_deg'], 10)

    def test_bad_crossing_points_mask_only_affected_metrics(self):
        valid = np.ones(len(self.f),dtype=bool)
        valid[(self.f>1.2)&(self.f<1.3)] = False
        m = self.metrics(point_valid=valid)
        self.assertIsNone(m['phase_margin_deg'])
        self.assertIsNotNone(m['gain_margin_db'])
        self.assertIn('unreliable samples',m['phase_margin_reason'])

    def test_sparse_and_ambiguous_phase_crossings_rejected(self):
        for hz,phase in [([1,10],[-100,-120]),([1,2],[-10,-170])]:
            l=np.array([2,.5])*np.exp(1j*np.deg2rad(phase))
            m=frf.bode_metrics(hz,l,l/(1+l),quality='valid')
            self.assertIsNone(m['phase_margin_deg'])
            self.assertIn('insufficiently resolved',m['phase_margin_reason'])

    def test_unsorted_data_and_quality_mask_stay_aligned(self):
        good=self.metrics()
        m=frf.bode_metrics(self.f[::-1],self.l[::-1],self.t[::-1],quality='valid',
                           point_valid=np.ones(len(self.f),dtype=bool))
        self.assertEqual(m,good)

    def test_singular_reconstruction_can_be_plotted_as_gap(self):
        l=self.l.copy();l[0]=np.nan+1j*np.nan
        m=frf.bode_metrics(self.f,l,self.t,quality='valid')
        self.assertIsNotNone(m['phase_margin_deg'])
        _,arrays=frf.bode_arrays(self.f,l,phase_anchor=1)
        self.assertTrue(np.isnan(arrays[0][1][0]))
        self.assertTrue(np.isfinite(arrays[0][1][1:]).all())

    def test_archive_legacy_is_unknown_and_invalid_round_trips(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'data.bode.nmixx'
            frf.save_bode(path,self.f,np.ones(len(self.f)),self.t,np.ones(len(self.f)),
                          0,1000,0,0,.4,1000,quality='invalid',quality_reason='oscillatory',
                          point_valid=np.ones(len(self.f),dtype=bool))
            # Test the real replot quality path without generating 4001-point PNGs.
            def plotted(f,l,s,t,title,dest,**kwargs):
                return frf.bode_metrics(f,l,t,**kwargs)
            with patch.object(frf,'plot_bode_a4',side_effect=plotted):
                m=next(iter(frf.replot(path,Path(tmp)/'plots').values()))
                self.assertEqual(m['phase_margin_reason'],'oscillatory')
                with np.load(path) as a:
                    legacy={k:a[k] for k in a.files if k not in ['quality','quality_reason','point_valid']}
                with path.open('wb') as h:np.savez_compressed(h,**legacy)
                m=next(iter(frf.replot(path,Path(tmp)/'plots').values()))
                self.assertEqual(m['quality'],'unknown')
                self.assertIsNone(m['phase_margin_deg'])

    def test_real_5000hz_branch_regression(self):
        path=Path(__file__).resolve().parents[1]/'docs/data/20261010_current_frf/Current-D-2026-10-10.bode.nmixx'
        with np.load(path) as a:
            for i in [12,13]:
                t=a['fbk'][i]/a['ref'][i]
                # Isolate the numerical branch fix, not physical validity.
                m=frf.bode_metrics(a['freq_hz'],t/(1-t),t,quality='valid')
                self.assertIsNotNone(m['phase_margin_deg'])
                self.assertLess(abs(m['phase_margin_deg']),30)
                m=frf.bode_metrics(a['freq_hz'],t/(1-t),t,quality='invalid',
                                   quality_reason='documented oscillation')
                self.assertIsNone(m['phase_margin_deg'])
                self.assertIsNone(m['gain_margin_db'])
                self.assertIsNone(m['relative_minus_3db_hz'])

    def test_all_archived_conditions_use_persisted_quality(self):
        root=Path(__file__).resolve().parents[1]/'docs/data/20261010_current_frf'
        for axis in ['D','Q']:
            with np.load(root/f'Current-{axis}-2026-10-10.bode.nmixx') as a:
                self.assertEqual(a['point_valid'].shape,(14,24))
                self.assertEqual(a['fit_residual'].shape,(14,24,3))
                self.assertEqual(a['fit_drift'].shape,(14,24,3))
                for i,bw in enumerate(a['bandwidth_hz']):
                    t=a['fbk'][i]/a['ref'][i]
                    m=frf.bode_metrics(a['freq_hz'],t/(1-t),t,quality=str(a['quality'][i]),
                                       quality_reason=str(a['quality_reason'][i]),
                                       point_valid=a['point_valid'][i])
                    if bw==5000:
                        self.assertEqual(a['quality'][i],'invalid')
                        self.assertIsNone(m['phase_margin_deg'])
                        self.assertIsNone(m['gain_margin_db'])
                        self.assertIsNone(m['relative_minus_3db_hz'])
                    else:
                        self.assertIsNotNone(m['phase_margin_deg'])
                        self.assertIsNotNone(m['relative_minus_3db_hz'])


class FitQualityTest(unittest.TestCase):
    def result(self,drift=False,noise=0):
        t=np.arange(20000)/20000
        ref=.1*np.sin(2*np.pi*200*t)
        fbk=.06*np.sin(2*np.pi*200*t-.4)
        if drift:fbk[len(fbk)//2:]*=2
        fbk+=noise*np.sin(2*np.pi*1733*t)
        out=.2*np.sin(2*np.pi*200*t-.2)
        raw=np.rint(np.column_stack([ref,ref,fbk,out])*1000).astype('<i2').tobytes()
        return frf.analyze(raw,200,.1,4,frf.CHANNELS['id'])

    def test_good_fit_requires_operator_linearity_confirmation(self):
        r=self.result()
        self.assertEqual(frf.sweep_quality([r])[0],'unknown')
        quality,_,valid=frf.sweep_quality([r],True)
        self.assertEqual(quality,'valid');self.assertTrue(valid[0])

    def test_nonstationarity_masks_point(self):
        _,_,valid=frf.sweep_quality([self.result(drift=True)],True)
        self.assertFalse(valid[0])

    def test_strong_off_tone_invalidates_even_verified_capture(self):
        quality,reason,_=frf.sweep_quality([self.result(noise=1)],True)
        self.assertEqual(quality,'invalid')
        self.assertIn('off-tone',reason)


if __name__ == '__main__':
    unittest.main()
