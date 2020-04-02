// Mozilla Public License 2.0
// If a copy of the MPL was not distributed with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// Copyright (C) LibreHardwareMonitor and Contributors
// All Rights Reserved

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace LibreHardwareMonitor.Hardware.CPU
{
    internal sealed class Amd17Cpu : AmdCpu
    {
        private readonly Processor _cpu;
        private int _sensorClock;
        private int _sensorMulti;
        private int _sensorPower;

        // counter, to create sensor index values
        private int _sensorTemperatures;
        private int _sensorVoltage;

        // librehardwaremonitor is very confused and thinks core 0 has 3 threads, which it doesn't
        // so just track performance data by thread id and add sensors as we go
        private static Dictionary<int, ThreadPerformanceData> _threadPerfData;
        private Sensor _totalInstructionsRetired;
        private Sensor _totalCoreClocks;
        private Sensor _averageIpc;
        private Sensor _l3Hitrate;
        private Sensor _l3HitBandwidth;
        private Sensor _l3MissLatency;
        private Sensor _l1DHitrate;
        private Sensor _l1DHitBandwidth;
        private Sensor _l2Hitrate;
        private Sensor _l2DcRefillBandwidth;
        private Sensor _l3DcRefillBandwidth;
        private Sensor _dramDcRefillBandwidth;
        private Sensor[] ccxL3HitBandwidth;

        // topology hardcode for CCX/CCD in 3950X
        private static int[] _ccxThreads = { 0, 8, 16, 24 };
        private static int[] _ccdThreads = { 0, 16 }; // this is a guess

        public Amd17Cpu(int processorIndex, CpuId[][] cpuId, ISettings settings) : base(processorIndex, cpuId, settings)
        {
            // add all numa nodes
            // Register ..1E_2, [10:8] + 1
            _cpu = new Processor(this);
            _threadPerfData = new Dictionary<int, ThreadPerformanceData>();
            _totalInstructionsRetired = new Sensor("Total instructions", 0, SensorType.Counter, this, _settings);
            _totalCoreClocks = new Sensor("Total Unhalted Clocks", 0, SensorType.Counter, this, _settings);
            _averageIpc = new Sensor("Average IPC", 0, SensorType.CounterRatio, this, _settings);
            _l3Hitrate = new Sensor("L3 Hitrate", 0, SensorType.Level, this, _settings);
            _l3HitBandwidth = new Sensor("L3 Hit Bandwidth", 0, SensorType.Throughput, this, _settings);
            _l3MissLatency = new Sensor("L3 Miss Latency (core clocks)", 0, SensorType.CounterRatio, this, _settings);
            _l1DHitrate = new Sensor("L1D Hitrate", 0, SensorType.Level, this, _settings);
            _l1DHitBandwidth = new Sensor("L1D Hit Bandwidth", 0, SensorType.Throughput, this, _settings);
            _l2Hitrate = new Sensor("L2 Hitrate", 0, SensorType.Level, this, _settings);
            _l2DcRefillBandwidth = new Sensor("L2 -> L1 Bandwidth", 0, SensorType.Throughput, this, _settings);
            _l3DcRefillBandwidth = new Sensor("L3 -> L1 Bandwidth", 0, SensorType.Throughput, this, _settings);
            _dramDcRefillBandwidth = new Sensor("DRAM -> L1 Bandwidth", 0, SensorType.Throughput, this, _settings);
            this.ActivateSensor(_totalInstructionsRetired);
            this.ActivateSensor(_totalCoreClocks);
            this.ActivateSensor(_averageIpc);
            this.ActivateSensor(_l3HitBandwidth);
            this.ActivateSensor(_l3MissLatency);
            this.ActivateSensor(_l3Hitrate);
            this.ActivateSensor(_l2Hitrate);
            this.ActivateSensor(_l1DHitrate);
            this.ActivateSensor(_dramDcRefillBandwidth);
            this.ActivateSensor(_l3DcRefillBandwidth);
            this.ActivateSensor(_l2DcRefillBandwidth);
            this.ActivateSensor(_l1DHitBandwidth);

            // add all numa nodes
            const int initialCoreId = 1_000_000_000;

            int coreId = 1;
            int lastCoreId = initialCoreId;

            // Ryzen 3000's skip some core ids.
            // So start at 1 and count upwards when the read core changes.
            foreach (CpuId[] cpu in cpuId.OrderBy(x => x[0].ExtData[0x1e, 1] & 0xFF))
            {
                CpuId thread = cpu[0];

                // coreID
                // Register ..1E_1, [7:0]
                int coreIdRead = (int)(thread.ExtData[0x1e, 1] & 0xff);

                // nodeID
                // Register ..1E_2, [7:0]
                int nodeId = (int)(thread.ExtData[0x1e, 2] & 0xff);

                _cpu.AppendThread(thread, nodeId, coreId);

                if (lastCoreId != initialCoreId && coreIdRead != lastCoreId)
                {
                    coreId++;
                }

                lastCoreId = coreIdRead;

                // Enable instructions retired counter
                ulong hwcrValue;
                Ring0.ReadMsr(HWCR, out hwcrValue, 1UL << thread.Thread);
                hwcrValue |= 1UL << 30;
                Ring0.WriteMsr(HWCR, hwcrValue, 1UL << thread.Thread);

                // 0x40 = dc access
                ulong L1RequestPerfCtl = GetPerfCtlValue(0x40, 0, true, true, false, false, true, false, 0, 0, false, false);
                // 0x41 = dc miss ("LS MAB Allocates by Type"). 0xb = all
                // setting umask 0x3 (excluding "dcPrefetcher") gives unrealistically high L1D hitrates. Not sure what this means yet
                ulong L1MissPerfCtl = GetPerfCtlValue(0x41, 0xB, true, true, false, false, true, false, 0, 0, false, false);
                // 0x64 = core to L2 cacheable request access status. umask 0xff = all, 0x9 = code or data miss
                // ulong L2RequestPerfCtl = GetPerfCtlValue(0x64, 0xFF, true, true, false, false, true, false, 0, 0, false, false);
                // ulong L2MissPerfCtl = GetPerfCtlValue(0x64, 0x9, true, true, false, false, true, false, 0, 0, false, false);
                ulong L1RefillFromDram = GetPerfCtlValue(0x43, 1 << 6 | 1 << 3, true, true, false, false, true, false, 0, 0, false, false);
                ulong L1RefillFromL2 = GetPerfCtlValue(0x43, 0x1, true, true, false, false, true, false, 0, 0, false, false);
                ulong L1RefillFromL3 = GetPerfCtlValue(0x43, 0x2, true, true, false, false, true, false, 0, 0, false, false);
                ulong L1RefillFromRemoteCCX = GetPerfCtlValue(0x43, 1 << 4, true, true, false, false, true, false, 0, 0, false, false);
                Ring0.WriteMsr(MSR_PERF_CTL_0, L1RequestPerfCtl, 1UL << thread.Thread);
                Ring0.WriteMsr(MSR_PERF_CTL_1, L1MissPerfCtl, 1UL << thread.Thread);
                Ring0.WriteMsr(MSR_PERF_CTL_2, L1RefillFromL2, 1UL << thread.Thread);
                Ring0.WriteMsr(MSR_PERF_CTL_3, L1RefillFromL3, 1UL << thread.Thread);
                Ring0.WriteMsr(MSR_PERF_CTL_4, L1RefillFromDram, 1UL << thread.Thread);
                Ring0.WriteMsr(MSR_PERF_CTL_5, L1RefillFromRemoteCCX, 1UL << thread.Thread);
                Ring0.WriteMsr(MSR_PERF_CTR_0, 0, 1UL << thread.Thread);
                Ring0.WriteMsr(MSR_PERF_CTR_1, 0, 1UL << thread.Thread);
                Ring0.WriteMsr(MSR_PERF_CTR_2, 0, 1UL << thread.Thread);
                Ring0.WriteMsr(MSR_PERF_CTR_3, 0, 1UL << thread.Thread);
                Ring0.WriteMsr(MSR_PERF_CTR_4, 0, 1UL << thread.Thread);
                Ring0.WriteMsr(MSR_PERF_CTR_5, 0, 1UL << thread.Thread);
            }

            // L3 per-CCX counters. Event 0x4 = All L3 Cache Requests
            // umask 0xFF = all L3 request types
            // umask 0x01 - this isn't explcitly documented, but AMD's PPR suggests
            // counting L3 misses with L3Event[0xFF0F000000400104] (umask = 0x1)
            ulong L3AccessPerfCtl = GetL3PerfCtlValue(0x04, 0xFF, true, 0xF, 0xFF);
            ulong L3MissPerfCtl = GetL3PerfCtlValue(0x04, 0x01, true, 0xF, 0xFF);
            ulong L3MissLatencyCtl = GetL3PerfCtlValue(0x90, 0, true, 0xF, 0xFF);
            ulong L3MissByRequestPerfCtl = GetL3PerfCtlValue(0x9A, 0x1F, true, 0xF, 0xFF);
            ccxL3HitBandwidth = new Sensor[_ccxThreads.Length];

            for (int i = 0; i < _ccxThreads.Length; i++)
            {
                Ring0.WriteMsr(MSR_L3_PERF_CTL_0, L3AccessPerfCtl, 1UL << _ccxThreads[i]);
                Ring0.WriteMsr(MSR_L3_PERF_CTL_1, L3MissPerfCtl, 1UL << _ccxThreads[i]);
                Ring0.WriteMsr(MSR_L3_PERF_CTL_2, L3MissLatencyCtl, 1UL << _ccxThreads[i]);
                Ring0.WriteMsr(MSR_L3_PERF_CTL_3, L3MissByRequestPerfCtl, 1UL << _ccxThreads[i]);
                Ring0.WriteMsr(MSR_L3_PERF_CTR_0, 0);
                Ring0.WriteMsr(MSR_L3_PERF_CTR_1, 0);
                Ring0.WriteMsr(MSR_L3_PERF_CTR_2, 0);
                Ring0.WriteMsr(MSR_L3_PERF_CTR_3, 0);
                ccxL3HitBandwidth[i] = new Sensor("CCX " + i + " L3 Hit Bandwidth", i + 1, SensorType.Throughput, this, _settings);
                ActivateSensor(ccxL3HitBandwidth[i]);
            }
            
            Update();
        }

        protected override uint[] GetMsrs()
        {
            return new[] { LEGACY_PERF_CTL_0, LEGACY_PERF_CTR_0, HWCR, MSR_PSTATE_0, COFVID_STATUS };
        }

        public override string GetReport()
        {
            StringBuilder r = new StringBuilder();
            r.Append(base.GetReport());
            r.Append("Ryzen");
            return r.ToString();
        }

        public override void Update()
        {
            base.Update();

            _cpu.UpdateSensors();
            foreach (NumaNode node in _cpu.Nodes)
            {
                NumaNode.UpdateSensors();

                foreach (Core c in node.Cores)
                {
                    c.UpdateSensors();
                }
            }

            // Read L3 counters
            ulong totalL3AccessCount = 0, totalL3MissCount = 0, totalL3MissLatency = 0, totalL3MissRequestCount = 0;
            for (int i = 0; i < _ccxThreads.Length; i++)
            {
                ulong ccxL3AccessCount, ccxL3MissCount, ccxL3MissLatency, ccxL3MissRequestCount;
                Ring0.ReadMsr(MSR_L3_PERF_CTR_0, out ccxL3AccessCount, 1UL << _ccxThreads[i]);
                Ring0.ReadMsr(MSR_L3_PERF_CTR_1, out ccxL3MissCount, 1UL << _ccxThreads[i]);
                Ring0.ReadMsr(MSR_L3_PERF_CTR_2, out ccxL3MissLatency, 1UL << _ccxThreads[i]);
                Ring0.ReadMsr(MSR_L3_PERF_CTR_3, out ccxL3MissRequestCount, 1UL << _ccxThreads[i]);
                Ring0.WriteMsr(MSR_L3_PERF_CTR_0, 0, 1UL << _ccxThreads[i]);
                Ring0.WriteMsr(MSR_L3_PERF_CTR_1, 0, 1UL << _ccxThreads[i]);
                Ring0.WriteMsr(MSR_L3_PERF_CTR_2, 0, 1UL << _ccxThreads[i]);
                Ring0.WriteMsr(MSR_L3_PERF_CTR_3, 0, 1UL << _ccxThreads[i]);
                totalL3AccessCount += ccxL3AccessCount;
                totalL3MissCount += ccxL3MissCount;
                totalL3MissLatency += ccxL3MissLatency;
                totalL3MissRequestCount += ccxL3MissRequestCount;
                ccxL3HitBandwidth[i].Value = ((float)ccxL3AccessCount - ccxL3MissCount) * 64;
            }

            _l3HitBandwidth.Value = ((float)totalL3AccessCount - totalL3MissCount) * 64;
            _l3Hitrate.Value = (1 - (float)totalL3MissCount / totalL3AccessCount) * 100;
            _l3MissLatency.Value = (float)(totalL3MissLatency * 16) / totalL3MissRequestCount;

            ulong _totalInstructionsRetiredCount = 0, _totalClockCount = 0;
            ulong totalDcAccessCount = 0, totalDcMissCount = 0, totalDcRefillFromL2Count = 0, totalDcRefillFromL3Count = 0, totalDcRefillFromDramCount = 0, totalDcRefillFromRemoteCCXCount = 0;
            foreach (KeyValuePair<int, ThreadPerformanceData> perfData in _threadPerfData)
            {
                _totalInstructionsRetiredCount += perfData.Value.instructionsRetiredCount;
                _totalClockCount += perfData.Value.coreClockCount;
                totalDcAccessCount += perfData.Value.dcAccessCount;
                totalDcMissCount += perfData.Value.dcMissCount;
                totalDcRefillFromL2Count += perfData.Value.dcRefillFromL2Count;
                totalDcRefillFromL3Count += perfData.Value.dcRefillFromL3Count;
                totalDcRefillFromDramCount += perfData.Value.dcRefillFromDramCount;
                totalDcRefillFromRemoteCCXCount += perfData.Value.dcRefillFromRemoteCCXCount;
            }

            _totalInstructionsRetired.Value = (float)((double)_totalInstructionsRetiredCount / 1000000000);
            _totalCoreClocks.Value = (float)((double)_totalClockCount / 1000000000);
            _averageIpc.Value = (float)_totalInstructionsRetiredCount / _totalClockCount;
            _l1DHitBandwidth.Value = (float)(totalDcAccessCount - totalDcMissCount) * 8;
            _l1DHitrate.Value = (1 - (float)totalDcMissCount / totalDcAccessCount) * 100;
            _l2DcRefillBandwidth.Value = (float)totalDcRefillFromL2Count * 64;
            _l3DcRefillBandwidth.Value = (float)totalDcRefillFromL3Count * 64;
            _dramDcRefillBandwidth.Value = (float)totalDcRefillFromDramCount * 64;
            _l2Hitrate.Value = (1 - (float)totalDcRefillFromL2Count / totalDcMissCount) * 100;
        }

        private class Processor
        {
            private readonly Sensor _coreTemperatureTctl;
            private readonly Sensor _coreTemperatureTdie;
            private readonly Sensor _coreTemperatureTctlTdie;
            private readonly Sensor[] _ccdTemperatures;
            private readonly Sensor _coreVoltage;
            private readonly Amd17Cpu _hw;
            private readonly Sensor _packagePower;
            private readonly Sensor _socVoltage;
            private Sensor _ccdsMaxTemperature;
            private Sensor _ccdsAverageTemperature;
            private DateTime _lastPwrTime = new DateTime(0);
            private uint _lastPwrValue;

            public Processor(Hardware hw)
            {
                _hw = (Amd17Cpu)hw;
                Nodes = new List<NumaNode>();

                _packagePower = new Sensor("Package Power", _hw._sensorPower++, SensorType.Power, _hw, _hw._settings);
                _coreTemperatureTctl = new Sensor("Core (Tctl)", _hw._sensorTemperatures++, SensorType.Temperature, _hw, _hw._settings);
                _coreTemperatureTdie = new Sensor("Core (Tdie)", _hw._sensorTemperatures++, SensorType.Temperature, _hw, _hw._settings);
                _coreTemperatureTctlTdie = new Sensor("Core (Tctl/Tdie)", _hw._sensorTemperatures++, SensorType.Temperature, _hw, _hw._settings);
                _ccdTemperatures = new Sensor[8]; // Hardcoded until there's a way to get max CCDs.
                _coreVoltage = new Sensor("Core (SVI2 TFN)", _hw._sensorVoltage++, SensorType.Voltage, _hw, _hw._settings);
                _socVoltage = new Sensor("SoC (SVI2 TFN)", _hw._sensorVoltage++, SensorType.Voltage, _hw, _hw._settings);

                _hw.ActivateSensor(_packagePower);
            }

            public List<NumaNode> Nodes { get; }

            public void UpdateSensors()
            {
                NumaNode node = Nodes[0];
                Core core = node?.Cores[0];
                CpuId cpu = core?.Threads[0];
                if (cpu == null)
                    return;


                ulong mask = Ring0.ThreadAffinitySet(1UL << cpu.Thread);

                // MSRC001_0299
                // TU [19:16]
                // ESU [12:8] -> Unit 15.3 micro Joule per increment
                // PU [3:0]
                Ring0.ReadMsr(MSR_PWR_UNIT, out uint _, out uint _);

                // MSRC001_029B
                // total_energy [31:0]
                DateTime sampleTime = DateTime.Now;
                Ring0.ReadMsr(MSR_PKG_ENERGY_STAT, out uint eax, out _);
                uint totalEnergy = eax;

                // THM_TCON_CUR_TMP
                // CUR_TEMP [31:21]
                Ring0.WritePciConfig(0x00, FAMILY_17H_PCI_CONTROL_REGISTER, F17H_M01H_THM_TCON_CUR_TMP);
                Ring0.ReadPciConfig(0x00, FAMILY_17H_PCI_CONTROL_REGISTER + 4, out uint temperature);

                // SVI0_TFN_PLANE0 [0]
                // SVI0_TFN_PLANE1 [1]
                Ring0.WritePciConfig(0x00, FAMILY_17H_PCI_CONTROL_REGISTER, F17H_M01H_SVI + 0x8);
                Ring0.ReadPciConfig(0x00, FAMILY_17H_PCI_CONTROL_REGISTER + 4, out uint smuSvi0Tfn);

                uint sviPlane0Offset;
                uint sviPlane1Offset;

                bool isZen2 = false;

                // TODO: find a better way because these will probably keep changing in the future.
                switch (cpu.Model)
                {
                    case 0x31: // Threadripper 3000.
                    {
                        sviPlane0Offset = F17H_M01H_SVI + 0x14;
                        sviPlane1Offset = F17H_M01H_SVI + 0x10;
                        isZen2 = true;
                        break;
                    }
                    case 0x71: // Zen 2.
                    {
                        sviPlane0Offset = F17H_M01H_SVI + 0x10;
                        sviPlane1Offset = F17H_M01H_SVI + 0xC;
                        isZen2 = true;
                        break;
                    }
                    default: // Zen and Zen+.
                    {
                        sviPlane0Offset = F17H_M01H_SVI + 0xC;
                        sviPlane1Offset = F17H_M01H_SVI + 0x10;
                        break;
                    }
                }

                // SVI0_PLANE0_VDDCOR [24:16]
                // SVI0_PLANE0_IDDCOR [7:0]
                Ring0.WritePciConfig(0x00, FAMILY_17H_PCI_CONTROL_REGISTER, sviPlane0Offset);
                Ring0.ReadPciConfig(0x00, FAMILY_17H_PCI_CONTROL_REGISTER + 4, out uint smuSvi0TelPlane0);

                // SVI0_PLANE1_VDDCOR [24:16]
                // SVI0_PLANE1_IDDCOR [7:0]
                Ring0.WritePciConfig(0x00, FAMILY_17H_PCI_CONTROL_REGISTER, sviPlane1Offset);
                Ring0.ReadPciConfig(0x00, FAMILY_17H_PCI_CONTROL_REGISTER + 4, out uint smuSvi0TelPlane1);

                Ring0.ThreadAffinitySet(mask);

                // power consumption
                // power.Value = (float) ((double)pu * 0.125);
                // esu = 15.3 micro Joule per increment
                if (_lastPwrTime.Ticks == 0)
                {
                    _lastPwrTime = sampleTime;
                    _lastPwrValue = totalEnergy;
                }

                // ticks diff
                TimeSpan time = sampleTime - _lastPwrTime;
                long pwr;
                if (_lastPwrValue <= totalEnergy)
                    pwr = totalEnergy - _lastPwrValue;
                else
                    pwr = (0xffffffff - _lastPwrValue) + totalEnergy;

                // update for next sample
                _lastPwrTime = sampleTime;
                _lastPwrValue = totalEnergy;

                double energy = 15.3e-6 * pwr;
                energy /= time.TotalSeconds;

                if (!double.IsNaN(energy))
                    _packagePower.Value = (float)energy;

                // current temp Bit [31:21]
                //If bit 19 of the Temperature Control register is set, there is an additional offset of 49 degrees C.
                bool tempOffsetFlag = (temperature & F17H_TEMP_OFFSET_FLAG) != 0;
                temperature = (temperature >> 21) * 125;

                float offset = 0.0f;

                // Offset table: https://github.com/torvalds/linux/blob/master/drivers/hwmon/k10temp.c#L78
                if (string.IsNullOrWhiteSpace(cpu.Name))
                    offset = 0;
                else if (cpu.Name.Contains("1600X") || cpu.Name.Contains("1700X") || cpu.Name.Contains("1800X"))
                    offset = -20.0f;
                else if (cpu.Name.Contains("Threadripper 19") || cpu.Name.Contains("Threadripper 29"))
                    offset = -27.0f;
                else if (cpu.Name.Contains("2700X"))
                    offset = -10.0f;

                float t = temperature * 0.001f;
                if (tempOffsetFlag)
                    t += -49.0f;

                if (offset < 0)
                {
                    _coreTemperatureTctl.Value = t;
                    _coreTemperatureTdie.Value = t + offset;

                    _hw.ActivateSensor(_coreTemperatureTctl);
                    _hw.ActivateSensor(_coreTemperatureTdie);
                }
                else
                {
                    // Zen 2 doesn't have an offset so Tdie and Tctl are the same.
                    _coreTemperatureTctlTdie.Value = t;
                    _hw.ActivateSensor(_coreTemperatureTctlTdie);
                }

                // Tested only on R5 3600 & Threadripper 3960X.
                if (isZen2)
                {
                    for (uint i = 0; i < _ccdTemperatures.Length; i++)
                    {
                        Ring0.WritePciConfig(0x00, FAMILY_17H_PCI_CONTROL_REGISTER, F17H_M70H_CCD1_TEMP + (i * 0x4));
                        Ring0.ReadPciConfig(0x00, FAMILY_17H_PCI_CONTROL_REGISTER + 4, out uint ccdTempData);

                        uint ccdTemp = ccdTempData & 0xFFF;
                        if (ccdTemp == 0)
                            continue;

                        if (_ccdTemperatures[i] == null)
                        {
                            _hw.ActivateSensor(_ccdTemperatures[i] = new Sensor($"Core CCD{i + 1} (Tdie)",
                                                                                _hw._sensorTemperatures++,
                                                                                SensorType.Temperature,
                                                                                _hw,
                                                                                _hw._settings));
                        }

                        _ccdTemperatures[i].Value = ((ccdTemp * 125) - 305000) * 0.001f;
                    }

                    Sensor[] activeCcds = _ccdTemperatures.Where(x => x != null).ToArray();
                    if (activeCcds.Length > 1)
                    {
                        // No need to get the max / average ccds temp if there is only one CCD.

                        if (_ccdsMaxTemperature == null)
                        {
                            _hw.ActivateSensor(_ccdsMaxTemperature = new Sensor("Core CCDs Max (Tdie)",
                                                                                _hw._sensorTemperatures++,
                                                                                SensorType.Temperature,
                                                                                _hw,
                                                                                _hw._settings));
                        }

                        if (_ccdsAverageTemperature == null)
                        {
                            _hw.ActivateSensor(_ccdsAverageTemperature = new Sensor("Core CCDs Average (Tdie)",
                                                                                    _hw._sensorTemperatures++,
                                                                                    SensorType.Temperature,
                                                                                    _hw,
                                                                                    _hw._settings));
                        }

                        _ccdsMaxTemperature.Value = activeCcds.Max(x => x.Value);
                        _ccdsAverageTemperature.Value = activeCcds.Average(x => x.Value);
                    }
                }

                // voltage
                const double vidStep = 0.00625;
                double vcc;
                uint svi0PlaneXVddCor;

                // Core (0x01).
                if ((smuSvi0Tfn & 0x01) == 0)
                {
                    svi0PlaneXVddCor = (smuSvi0TelPlane0 >> 16) & 0xff;
                    vcc = 1.550 - vidStep * svi0PlaneXVddCor;
                    _coreVoltage.Value = (float)vcc;

                    _hw.ActivateSensor(_coreVoltage);
                }

                // SoC (0x02), not every Zen cpu has this voltage.
                if (cpu.Model == 0x71 || cpu.Model == 0x31 || (smuSvi0Tfn & 0x02) == 0)
                {
                    svi0PlaneXVddCor = (smuSvi0TelPlane1 >> 16) & 0xff;
                    vcc = 1.550 - vidStep * svi0PlaneXVddCor;
                    _socVoltage.Value = (float)vcc;

                    _hw.ActivateSensor(_socVoltage);
                }
            }

            public void AppendThread(CpuId thread, int numaId, int coreId)
            {
                NumaNode node = null;
                foreach (NumaNode n in Nodes)
                {
                    if (n.NodeId == numaId)
                    {
                        node = n;
                        break;
                    }
                }

                if (node == null)
                {
                    node = new NumaNode(_hw, numaId);
                    Nodes.Add(node);
                }

                if (thread != null)
                    node.AppendThread(thread, coreId);
            }
        }

        private class NumaNode
        {
            private readonly Amd17Cpu _hw;

            public NumaNode(Hardware hw, int id)
            {
                Cores = new List<Core>();
                NodeId = id;
                _hw = (Amd17Cpu)hw;
            }

            public List<Core> Cores { get; }

            public int NodeId { get; }

            public void AppendThread(CpuId thread, int coreId)
            {
                Core core = null;
                foreach (Core c in Cores)
                {
                    if (c.CoreId == coreId)
                        core = c;
                }

                if (core == null)
                {
                    core = new Core(_hw, coreId);
                    Cores.Add(core);
                }

                if (thread != null)
                    core.Threads.Add(thread);
            }

            public static void UpdateSensors()
            { }
        }

        private class Core
        {
            private readonly Sensor _clock;
            private readonly Sensor _multiplier;
            private readonly Sensor _power;
            private readonly Sensor _vcore;
            private readonly Sensor _instructions;
            private DateTime _lastPwrTime = new DateTime(0);
            private uint _lastPwrValue;
            private ulong[] _lastInstrRetiredValues;
            private Amd17Cpu cpuhw;

            public Core(Hardware hw, int id)
            {
                Threads = new List<CpuId>();
                CoreId = id;
                cpuhw = (Amd17Cpu)hw;
                _lastInstrRetiredValues = null;
                _clock = new Sensor("Core #" + CoreId, cpuhw._sensorClock++, SensorType.Clock, cpuhw, cpuhw._settings);
                _multiplier = new Sensor("Core #" + CoreId, cpuhw._sensorMulti++, SensorType.Factor, cpuhw, cpuhw._settings);
                _power = new Sensor("Core #" + CoreId + " (SMU)", cpuhw._sensorPower++, SensorType.Power, cpuhw, cpuhw._settings);
                _vcore = new Sensor("Core #" + CoreId + " VID", cpuhw._sensorVoltage++, SensorType.Voltage, cpuhw, cpuhw._settings);

                cpuhw.ActivateSensor(_clock);
                cpuhw.ActivateSensor(_multiplier);
                cpuhw.ActivateSensor(_power);
                cpuhw.ActivateSensor(_vcore);
            }

            public int CoreId { get; }

            public List<CpuId> Threads { get; }

            public void UpdateSensors()
            {
                // CPUID cpu = threads.FirstOrDefault();
                CpuId cpu = Threads[0];
                if (cpu == null)
                    return;


                ulong mask = Ring0.ThreadAffinitySet(1UL << cpu.Thread);

                // MSRC001_0299
                // TU [19:16]
                // ESU [12:8] -> Unit 15.3 micro Joule per increment
                // PU [3:0]
                Ring0.ReadMsr(MSR_PWR_UNIT, out _, out _);

                // MSRC001_029A
                // total_energy [31:0]
                DateTime sampleTime = DateTime.Now;
                uint eax;
                Ring0.ReadMsr(MSR_CORE_ENERGY_STAT, out eax, out _);
                uint totalEnergy = eax;

                // MSRC001_0293
                // CurHwPstate [24:22]
                // CurCpuVid [21:14]
                // CurCpuDfsId [13:8]
                // CurCpuFid [7:0]
                Ring0.ReadMsr(MSR_HARDWARE_PSTATE_STATUS, out eax, out _);
                int curCpuVid = (int)((eax >> 14) & 0xff);
                int curCpuDfsId = (int)((eax >> 8) & 0x3f);
                int curCpuFid = (int)(eax & 0xff);

                // MSRC001_0064 + x
                // IddDiv [31:30]
                // IddValue [29:22]
                // CpuVid [21:14]
                // CpuDfsId [13:8]
                // CpuFid [7:0]
                // Ring0.ReadMsr(MSR_PSTATE_0 + (uint)CurHwPstate, out eax, out edx);
                // int IddDiv = (int)((eax >> 30) & 0x03);
                // int IddValue = (int)((eax >> 22) & 0xff);
                // int CpuVid = (int)((eax >> 14) & 0xff);
                Ring0.ThreadAffinitySet(mask);

                // clock
                // CoreCOF is (Core::X86::Msr::PStateDef[CpuFid[7:0]] / Core::X86::Msr::PStateDef[CpuDfsId]) * 200
                _clock.Value = (float)(curCpuFid / (double)curCpuDfsId * 200.0);

                // multiplier
                _multiplier.Value = (float)(curCpuFid / (double)curCpuDfsId * 2.0);

                // Voltage
                const double vidStep = 0.00625;
                double vcc = 1.550 - vidStep * curCpuVid;
                _vcore.Value = (float)vcc;

                // power consumption
                // power.Value = (float) ((double)pu * 0.125);
                // esu = 15.3 micro Joule per increment
                if (_lastPwrTime.Ticks == 0)
                {
                    _lastPwrTime = sampleTime;
                    _lastPwrValue = totalEnergy;
                }

                // ticks diff
                TimeSpan time = sampleTime - _lastPwrTime;
                long pwr;
                if (_lastPwrValue <= totalEnergy)
                    pwr = totalEnergy - _lastPwrValue;
                else
                    pwr = (0xffffffff - _lastPwrValue) + totalEnergy;

                // update for next sample
                _lastPwrTime = sampleTime;
                _lastPwrValue = totalEnergy;

                double energy = 15.3e-6 * pwr;
                energy /= time.TotalSeconds;

                if (!double.IsNaN(energy))
                    _power.Value = (float)energy;

                for (int i = 0;i < Threads.Count; i++)
                {
                    ulong instrCount, coreClockCount;
                    Ring0.ReadMsr(MSR_INSTR_RETIRED, out instrCount, 1UL << Threads[i].Thread);
                    Ring0.ReadMsr(MSR_APERF, out coreClockCount);

                    ulong dcAccessCount, dcMissCount, dcRefillFromL2Count, dcRefillFromL3Count, dcRefillFromDramCount, dcRefillFromRemoteCCXCount;
                    Ring0.ReadMsr(MSR_PERF_CTR_0, out dcAccessCount, 1UL << Threads[i].Thread);
                    Ring0.ReadMsr(MSR_PERF_CTR_1, out dcMissCount, 1UL << Threads[i].Thread);
                    Ring0.ReadMsr(MSR_PERF_CTR_2, out dcRefillFromL2Count, 1UL << Threads[i].Thread);
                    Ring0.ReadMsr(MSR_PERF_CTR_3, out dcRefillFromL3Count, 1UL << Threads[i].Thread);
                    Ring0.ReadMsr(MSR_PERF_CTR_4, out dcRefillFromDramCount, 1UL << Threads[i].Thread);
                    Ring0.ReadMsr(MSR_PERF_CTR_5, out dcRefillFromRemoteCCXCount, 1UL << Threads[i].Thread);
                    Ring0.WriteMsr(MSR_PERF_CTR_0, 0, 1UL << Threads[i].Thread);
                    Ring0.WriteMsr(MSR_PERF_CTR_1, 0, 1UL << Threads[i].Thread);
                    Ring0.WriteMsr(MSR_PERF_CTR_2, 0, 1UL << Threads[i].Thread);
                    Ring0.WriteMsr(MSR_PERF_CTR_3, 0, 1UL << Threads[i].Thread);
                    Ring0.WriteMsr(MSR_PERF_CTR_4, 0, 1UL << Threads[i].Thread);
                    Ring0.WriteMsr(MSR_PERF_CTR_5, 0, 1UL << Threads[i].Thread);

                    ThreadPerformanceData threadPerformanceData;
                    if (_threadPerfData.TryGetValue(Threads[i].Thread, out threadPerformanceData))
                    {
                        threadPerformanceData.instructionsRetiredCount = instrCount > threadPerformanceData.lastInstructionCount ? instrCount - threadPerformanceData.lastInstructionCount : instrCount;
                        threadPerformanceData.coreClockCount = coreClockCount > threadPerformanceData.lastCoreClockCount ? coreClockCount - threadPerformanceData.lastCoreClockCount : coreClockCount;
                        threadPerformanceData.instructionsRetired.Value = (float)((double)threadPerformanceData.instructionsRetiredCount / 1000000000);
                        threadPerformanceData.threadIpc.Value = (float)threadPerformanceData.instructionsRetiredCount / threadPerformanceData.coreClockCount;
                        threadPerformanceData.aperf.Value = (float)((double)threadPerformanceData.coreClockCount / 1000000000);
                        threadPerformanceData.l1dHitrate.Value = (1 - (float)dcMissCount / dcAccessCount) * 100;
                    }
                    else
                    {
                        threadPerformanceData = new ThreadPerformanceData();
                        threadPerformanceData.instructionsRetired = new Sensor("Thread " + Threads[i].Thread + " Instructions", Threads[i].Thread, true, SensorType.Counter, cpuhw, null, cpuhw._settings);
                        threadPerformanceData.threadIpc = new Sensor("Thread " + Threads[i].Thread + " IPC", Threads[i].Thread, true, SensorType.CounterRatio, cpuhw, null, cpuhw._settings);
                        threadPerformanceData.aperf = new Sensor("Thread " + Threads[i].Thread + " APERF", Threads[i].Thread, true, SensorType.Counter, cpuhw, null, cpuhw._settings);
                        threadPerformanceData.l1dHitrate = new Sensor("Thread " + Threads[i].Thread + " L1D Hitrate", Threads[i].Thread, true, SensorType.Level, cpuhw, null, cpuhw._settings);
                        threadPerformanceData.l2Hitrate = new Sensor("Thread " + Threads[i].Thread + " L2 Hitrate", Threads[i].Thread, true, SensorType.Level, cpuhw, null, cpuhw._settings);
                        cpuhw.ActivateSensor(threadPerformanceData.instructionsRetired);
                        cpuhw.ActivateSensor(threadPerformanceData.threadIpc);
                        cpuhw.ActivateSensor(threadPerformanceData.aperf);
                        cpuhw.ActivateSensor(threadPerformanceData.l1dHitrate);
                        cpuhw.ActivateSensor(threadPerformanceData.l2Hitrate);

                        _threadPerfData.Add(Threads[i].Thread, threadPerformanceData);
                    }

                    threadPerformanceData.lastInstructionCount = instrCount;
                    threadPerformanceData.lastCoreClockCount = coreClockCount;
                    threadPerformanceData.dcAccessCount = dcAccessCount;
                    threadPerformanceData.dcMissCount = dcMissCount;
                    threadPerformanceData.dcRefillFromL2Count = dcRefillFromL2Count;
                    threadPerformanceData.dcRefillFromL3Count = dcRefillFromL3Count;
                    threadPerformanceData.dcRefillFromDramCount = dcRefillFromDramCount;
                    threadPerformanceData.dcRefillFromRemoteCCXCount = dcRefillFromRemoteCCXCount;
                }
            }
        }

        private class ThreadPerformanceData
        {
            public Sensor instructionsRetired;
            public Sensor aperf;
            public Sensor threadIpc;
            public Sensor l1dHitrate;
            public Sensor l2Hitrate;
            // last read from MSRC000_00E9, subtract from next read to get instructions retired over interval
            public ulong lastInstructionCount;
            // ditto for unhalted clocks
            public ulong lastCoreClockCount;

            // actual counts over interval
            public ulong instructionsRetiredCount;
            public ulong coreClockCount;
            public ulong dcAccessCount;
            public ulong dcMissCount;
            public ulong dcRefillFromL2Count;
            public ulong dcRefillFromL3Count;
            public ulong dcRefillFromDramCount;
            public ulong dcRefillFromRemoteCCXCount;
        }

        /// <summary>
        /// Get perf ctl value
        /// </summary>
        /// <param name="perfEvent">Low 8 bits of performance event</param>
        /// <param name="umask">perf event umask</param>
        /// <param name="usr">count user events?</param>
        /// <param name="os">count os events?</param>
        /// <param name="edge">only increment on transition</param>
        /// <param name="interrupt">generate apic interrupt on overflow</param>
        /// <param name="enable">enable perf ctr</param>
        /// <param name="invert">invert cmask</param>
        /// <param name="cmask">0 = increment by event count. >0 = increment by 1 if event count in clock cycle >= cmask</param>
        /// <param name="perfEventHi">high 4 bits of performance event</param>
        /// <param name="guest">count guest events if virtualization enabled</param>
        /// <param name="host">count host events if virtualization enabled</param>
        /// <returns>value for perf ctl msr</returns>
        private ulong GetPerfCtlValue(byte perfEvent, byte umask, bool usr, bool os, bool edge, bool interrupt, bool enable, bool invert, byte cmask, byte perfEventHi, bool guest, bool host)
        {
            return (ulong)perfEvent |
                (ulong)umask << 8 |
                (usr ? 1UL : 0UL) << 16 |
                (os ? 1UL : 0UL) << 17 |
                (edge ? 1UL : 0UL) << 18 |
                (interrupt ? 1UL : 0UL) << 20 |
                (enable ? 1UL : 0UL) << 22 |
                (invert ? 1UL : 0UL) << 23 |
                (ulong)cmask << 24 |
                (ulong)perfEventHi << 32 |
                (host ? 1UL : 0UL) << 40 |
                (guest ? 1UL : 0UL) << 41;
        }

        /// <summary>
        /// Get L3 perf ctl value
        /// </summary>
        /// <param name="perfEvent">Event select</param>
        /// <param name="umask">unit mask</param>
        /// <param name="enable">enable perf counter</param>
        /// <param name="sliceMask">L3 slice select. bit 0 = slice 0, etc. 4 slices in ccx</param>
        /// <param name="threadMask">Thread select. bit 0 = c0t0, bit 1 = c0t1, bit 2 = c1t0, etc. Up to 8 threads in ccx</param>
        /// <returns>value to put in ChL3PmcCfg</returns>
        private ulong GetL3PerfCtlValue(byte perfEvent, byte umask, bool enable, byte sliceMask, byte threadMask)
        {
            return (ulong)perfEvent |
                (ulong)umask << 8 |
                (enable ? 1UL : 0UL) << 22 |
                (ulong)sliceMask << 48 |
                (ulong)threadMask << 56;
        }

        /// <summary>
        /// Get data fabric performance event select MSR value
        /// </summary>
        /// <param name="perfEventLow">Low 8 bits of performance event select</param>
        /// <param name="umask">unit mask</param>
        /// <param name="enable">enable perf counter</param>
        /// <param name="perfEventHi">bits 8-11 of performance event select</param>
        /// <param name="perfEventHi1">high 2 bits (12-13) of performance event select</param>
        /// <returns>value to put in DF_PERF_CTL</returns>
        private ulong GetDFPerfCtlValue(byte perfEventLow, byte umask, bool enable, byte perfEventHi, byte perfEventHi1)
        {
            return (ulong)perfEventHi |
                (ulong)umask << 8 |
                (enable ? 1UL : 0UL) << 22 |
                (ulong)perfEventHi << 32 |
                (ulong)perfEventHi1 << 59;
        }

        // ReSharper disable InconsistentNaming
        private const uint COFVID_STATUS = 0xC0010071;
        private const uint F17H_M01H_SVI = 0x0005A000;
        private const uint F17H_M01H_THM_TCON_CUR_TMP = 0x00059800;
        private const uint F17H_M70H_CCD1_TEMP = 0x00059954;
        private const uint F17H_TEMP_OFFSET_FLAG = 0x80000;
        private const uint FAMILY_17H_PCI_CONTROL_REGISTER = 0x60;
        private const uint HWCR = 0xC0010015;
        private const uint MSR_CORE_ENERGY_STAT = 0xC001029A;
        private const uint MSR_HARDWARE_PSTATE_STATUS = 0xC0010293;
        private const uint MSR_PKG_ENERGY_STAT = 0xC001029B;
        private const uint MSR_PSTATE_0 = 0xC0010064;
        private const uint MSR_PWR_UNIT = 0xC0010299;
        private const uint MSR_INSTR_RETIRED = 0xC00000E9;
        private const uint MSR_APERF = 0x000000E8;
        private const uint MSR_MPERF = 0x000000E7;
        private const uint MSR_TSC = 0x00000010;
        private const uint MSR_PERF_CTR_0 = 0xC0010201;
        private const uint MSR_PERF_CTR_1 = 0xC0010203;
        private const uint MSR_PERF_CTR_2 = 0xC0010205;
        private const uint MSR_PERF_CTR_3 = 0xC0010207;
        private const uint MSR_PERF_CTR_4 = 0xC0010209;
        private const uint MSR_PERF_CTR_5 = 0xC001020B;
        private const uint MSR_PERF_CTL_0 = 0xC0010200;
        private const uint MSR_PERF_CTL_1 = 0xC0010202;
        private const uint MSR_PERF_CTL_2 = 0xC0010204;
        private const uint MSR_PERF_CTL_3 = 0xC0010206;
        private const uint MSR_PERF_CTL_4 = 0xC0010208;
        private const uint MSR_PERF_CTL_5 = 0xC001020A;
        private const uint MSR_L3_PERF_CTL_0 = 0xC0010230;
        private const uint MSR_L3_PERF_CTL_1 = 0xC0010232;
        private const uint MSR_L3_PERF_CTL_2 = 0xC0010234;
        private const uint MSR_L3_PERF_CTL_3 = 0xC0010236;
        private const uint MSR_L3_PERF_CTL_4 = 0xC0010238;
        private const uint MSR_L3_PERF_CTL_5 = 0xC001023A;
        private const uint MSR_L3_PERF_CTR_0 = 0xC0010231;
        private const uint MSR_L3_PERF_CTR_1 = 0xC0010233;
        private const uint MSR_L3_PERF_CTR_2 = 0xC0010235;
        private const uint MSR_L3_PERF_CTR_3 = 0xC0010237;
        private const uint MSR_L3_PERF_CTR_4 = 0xC0010239;
        private const uint MSR_L3_PERF_CTR_5 = 0xC001023B;
        private const uint MSR_DF_PERF_CTL_0 = 0xC0010240;
        private const uint MSR_DF_PERF_CTL_1 = 0xC0010242;
        private const uint MSR_DF_PERF_CTL_2 = 0xC0010244;
        private const uint MSR_DF_PERF_CTL_3 = 0xC0010246;
        private const uint MSR_DF_PERF_CTR_0 = 0xC0010241;
        private const uint MSR_DF_PERF_CTR_1 = 0xC0010243;
        private const uint MSR_DF_PERF_CTR_2 = 0xC0010245;
        private const uint MSR_DF_PERF_CTR_3 = 0xC0010247;
        private const uint LEGACY_PERF_CTL_0 = 0xC0010000;
        private const uint LEGACY_PERF_CTR_0 = 0xC0010004;
        // ReSharper restore InconsistentNaming
    }
}
