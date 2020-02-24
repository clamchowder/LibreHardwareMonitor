// Mozilla Public License 2.0
// If a copy of the MPL was not distributed with this file, You can obtain one at http://mozilla.org/MPL/2.0/.
// Copyright (C) LibreHardwareMonitor and Contributors
// All Rights Reserved

using System;
using System.Diagnostics.CodeAnalysis;
using System.Globalization;
using System.Text;

namespace LibreHardwareMonitor.Hardware.CPU
{
    internal sealed class IntelCpu : GenericCpu
    {
        private readonly Sensor _busClock;
        private readonly Sensor _coreAvg;
        private readonly Sensor[] _coreClocks;
        private readonly Sensor _coreMax;
        private readonly Sensor[] _coreTemperatures;
        private readonly Sensor[] _distToTjMaxTemperatures;

        private readonly uint[] _energyStatusMsrs = { MSR_PKG_ENERY_STATUS, MSR_PP0_ENERY_STATUS, MSR_PP1_ENERY_STATUS, MSR_DRAM_ENERGY_STATUS };
        private readonly float _energyUnitMultiplier;
        private readonly uint[] _lastEnergyConsumed;
        private readonly DateTime[] _lastEnergyTime;

        private readonly MicroArchitecture _microArchitecture;
        private readonly Sensor _packageTemperature;
        private readonly Sensor[] _powerSensors;
        private readonly double _timeStampCounterMultiplier;

        private readonly Sensor[] retiredInstructions;
        private readonly Sensor[] unhaltedThreadClocks;
        private readonly Sensor[] IPC;
        private readonly Sensor totalRetiredInstructions;
        private readonly Sensor totalUnhaltedThreadClocks;
        private readonly Sensor averageIPC;
        private readonly Sensor[] L1CacheLoadBandwidth;
        private readonly Sensor[] L2CacheLoadBandwidth;
        private readonly Sensor[] L1DHitRate;
        private readonly Sensor[] L2HitRate;
        private readonly Sensor totalL1CacheLoadBandwidth;
        private readonly Sensor totalL2CacheLoadBandwidth;
        private readonly Sensor averageL1DHitRate;
        private readonly Sensor averageL2HitRate;
        private readonly Sensor dramReadBandwidth;
        private readonly Sensor dramWriteBandwidth;
        private readonly Sensor dramBandwidth;
        private readonly Sensor[] cboxL3Hits;
        private readonly Sensor[] cboxL3Lookups;
        private readonly Sensor[] cboxL3Hitrate;
        private readonly Sensor cboxL3HitBandwidth;
        private readonly Sensor averageL3HitRate;

        private bool pmuInitialized = false;
        private bool pmcsInitialized = false;
        private bool uncorePmuInitialized = false;
        private ulong imcBar = 0;
        private uint lastDramReadsValue = 0;
        private uint lastDramWritesValue = 0;
        private uint cboBankCount = 0;

        public IntelCpu(int processorIndex, CpuId[][] cpuId, ISettings settings) : base(processorIndex, cpuId, settings)
        {
            // set tjMax
            float[] tjMax;
            switch (_family)
            {
                case 0x06:
                {
                    switch (_model)
                    {
                        case 0x0F: // Intel Core 2 (65nm)
                            _microArchitecture = MicroArchitecture.Core;
                            switch (_stepping)
                            {
                                case 0x06: // B2
                                    switch (_coreCount)
                                    {
                                        case 2:
                                            tjMax = Floats(80 + 10);
                                            break;
                                        case 4:
                                            tjMax = Floats(90 + 10);
                                            break;
                                        default:
                                            tjMax = Floats(85 + 10);
                                            break;
                                    }
                                    break;
                                case 0x0B: // G0
                                    tjMax = Floats(90 + 10);
                                    break;
                                case 0x0D: // M0
                                    tjMax = Floats(85 + 10);
                                    break;
                                default:
                                    tjMax = Floats(85 + 10);
                                    break;
                            }
                            break;
                        case 0x17: // Intel Core 2 (45nm)
                            _microArchitecture = MicroArchitecture.Core;
                            tjMax = Floats(100);
                            break;
                        case 0x1C: // Intel Atom (45nm)
                            _microArchitecture = MicroArchitecture.Atom;
                            switch (_stepping)
                            {
                                case 0x02: // C0
                                    tjMax = Floats(90);
                                    break;
                                case 0x0A: // A0, B0
                                    tjMax = Floats(100);
                                    break;
                                default:
                                    tjMax = Floats(90);
                                    break;
                            }
                            break;
                        case 0x1A: // Intel Core i7 LGA1366 (45nm)
                        case 0x1E: // Intel Core i5, i7 LGA1156 (45nm)
                        case 0x1F: // Intel Core i5, i7
                        case 0x25: // Intel Core i3, i5, i7 LGA1156 (32nm)
                        case 0x2C: // Intel Core i7 LGA1366 (32nm) 6 Core
                        case 0x2E: // Intel Xeon Processor 7500 series (45nm)
                        case 0x2F: // Intel Xeon Processor (32nm)
                            _microArchitecture = MicroArchitecture.Nehalem;
                            tjMax = GetTjMaxFromMsr();
                            break;
                        case 0x2A: // Intel Core i5, i7 2xxx LGA1155 (32nm)
                        case 0x2D: // Next Generation Intel Xeon, i7 3xxx LGA2011 (32nm)
                            _microArchitecture = MicroArchitecture.SandyBridge;
                            tjMax = GetTjMaxFromMsr();
                            break;
                        case 0x3A: // Intel Core i5, i7 3xxx LGA1155 (22nm)
                        case 0x3E: // Intel Core i7 4xxx LGA2011 (22nm)
                            _microArchitecture = MicroArchitecture.IvyBridge;
                            tjMax = GetTjMaxFromMsr();
                            break;
                        case 0x3C: // Intel Core i5, i7 4xxx LGA1150 (22nm)
                        case 0x3F: // Intel Xeon E5-2600/1600 v3, Core i7-59xx
                        // LGA2011-v3, Haswell-E (22nm)
                        case 0x45: // Intel Core i5, i7 4xxxU (22nm)
                        case 0x46:
                            _microArchitecture = MicroArchitecture.Haswell;
                            tjMax = GetTjMaxFromMsr();
                            break;
                        case 0x3D: // Intel Core M-5xxx (14nm)
                        case 0x47: // Intel i5, i7 5xxx, Xeon E3-1200 v4 (14nm)
                        case 0x4F: // Intel Xeon E5-26xx v4
                        case 0x56: // Intel Xeon D-15xx
                            _microArchitecture = MicroArchitecture.Broadwell;
                            tjMax = GetTjMaxFromMsr();
                            break;
                        case 0x36: // Intel Atom S1xxx, D2xxx, N2xxx (32nm)
                            _microArchitecture = MicroArchitecture.Atom;
                            tjMax = GetTjMaxFromMsr();
                            break;
                        case 0x37: // Intel Atom E3xxx, Z3xxx (22nm)
                        case 0x4A:
                        case 0x4D: // Intel Atom C2xxx (22nm)
                        case 0x5A:
                        case 0x5D:
                            _microArchitecture = MicroArchitecture.Silvermont;
                            tjMax = GetTjMaxFromMsr();
                            break;
                        case 0x4E:
                        case 0x5E: // Intel Core i5, i7 6xxxx LGA1151 (14nm)
                        case 0x55: // Intel Core X i7, i9 7xxx LGA2066 (14nm)
                            _microArchitecture = MicroArchitecture.Skylake;
                            tjMax = GetTjMaxFromMsr();
                            break;
                        case 0x4C: // Intel Airmont (Cherry Trail, Braswell)
                            _microArchitecture = MicroArchitecture.Airmont;
                            tjMax = GetTjMaxFromMsr();
                            break;
                        case 0x8E: // Intel Core i5, i7 7xxxx (14nm) (Kaby Lake) and 8xxxx (14nm++) (Coffee Lake)
                        case 0x9E: 
                            _microArchitecture = MicroArchitecture.KabyLake;
                            tjMax = GetTjMaxFromMsr();
                            break;
                        case 0x5C: // Goldmont (Apollo Lake)
                        case 0x5F: // (Denverton)
                            _microArchitecture = MicroArchitecture.Goldmont;
                            tjMax = GetTjMaxFromMsr();
                            break;
                        case 0x7A: // Goldmont plus (Gemini Lake)
                            _microArchitecture = MicroArchitecture.GoldmontPlus;
                            tjMax = GetTjMaxFromMsr();
                            break;
                        case 0x66: // Intel Core i3 8xxx (10nm) (Cannon Lake)
                            _microArchitecture = MicroArchitecture.CannonLake;
                            tjMax = GetTjMaxFromMsr();
                            break;
                        case 0x7D: // Intel Core i3, i5, i7 10xxx (10nm) (Ice Lake)
                        case 0x7E:
                        case 0x6A: // Ice Lake server
                        case 0x6C:
                            _microArchitecture = MicroArchitecture.IceLake;
                            tjMax = GetTjMaxFromMsr();
                            break;
                        case 0x86: // Tremont (10nm) (Elkhart Lake, Skyhawk Lake)
                            _microArchitecture = MicroArchitecture.Tremont;
                            tjMax = GetTjMaxFromMsr();
                            break;
                        case 0x8C: // Tiger Lake (10nm)
                        case 0x8D:
                            _microArchitecture = MicroArchitecture.TigerLake;
                            tjMax = GetTjMaxFromMsr();
                            break;
                        default:
                            _microArchitecture = MicroArchitecture.Unknown;
                            tjMax = Floats(100);
                            break;
                    }
                }
                break;
                case 0x0F:
                {
                    switch (_model)
                    {
                        case 0x00: // Pentium 4 (180nm)
                        case 0x01: // Pentium 4 (130nm)
                        case 0x02: // Pentium 4 (130nm)
                        case 0x03: // Pentium 4, Celeron D (90nm)
                        case 0x04: // Pentium 4, Pentium D, Celeron D (90nm)
                        case 0x06: // Pentium 4, Pentium D, Celeron D (65nm)
                            _microArchitecture = MicroArchitecture.NetBurst;
                            tjMax = Floats(100);
                            break;
                        default:
                            _microArchitecture = MicroArchitecture.Unknown;
                            tjMax = Floats(100);
                            break;
                    }
                }
                break;
                default:
                    _microArchitecture = MicroArchitecture.Unknown;
                    tjMax = Floats(100);
                break;
            }
            // set timeStampCounterMultiplier
            switch (_microArchitecture)
            {
                case MicroArchitecture.NetBurst:
                case MicroArchitecture.Atom:
                case MicroArchitecture.Core:
                {
                    if (Ring0.ReadMsr(IA32_PERF_STATUS, out uint _, out uint edx))
                    {
                        _timeStampCounterMultiplier = ((edx >> 8) & 0x1f) + 0.5 * ((edx >> 14) & 1);
                    }
                }
                break;
                case MicroArchitecture.Nehalem:
                case MicroArchitecture.SandyBridge:
                case MicroArchitecture.IvyBridge:
                case MicroArchitecture.Haswell:
                case MicroArchitecture.Broadwell:
                case MicroArchitecture.Silvermont:
                case MicroArchitecture.Skylake:
                case MicroArchitecture.Airmont:
                case MicroArchitecture.Goldmont:
                case MicroArchitecture.KabyLake:
                case MicroArchitecture.GoldmontPlus:
                case MicroArchitecture.CannonLake:
                case MicroArchitecture.IceLake:
                case MicroArchitecture.TigerLake:
                case MicroArchitecture.Tremont:
                {
                    if (Ring0.ReadMsr(MSR_PLATFORM_INFO, out uint eax, out uint _))
                    {
                        _timeStampCounterMultiplier = (eax >> 8) & 0xff;
                    }
                }
                break;
                default:
                    _timeStampCounterMultiplier = 0;
                    break;
            }

            int coreSensorId = 0;

            // check if processor supports a digital thermal sensor at core level
            if (cpuId[0][0].Data.GetLength(0) > 6 && (cpuId[0][0].Data[6, 0] & 1) != 0 && _microArchitecture != MicroArchitecture.Unknown)
            {
                _coreTemperatures = new Sensor[_coreCount];
                for (int i = 0; i < _coreTemperatures.Length; i++)
                {
                    _coreTemperatures[i] = new Sensor(CoreString(i),
                                                      coreSensorId,
                                                      SensorType.Temperature,
                                                      this,
                                                      new[]
                                                      {
                                                          new ParameterDescription("TjMax [°C]", "TjMax temperature of the core sensor.\n" + "Temperature = TjMax - TSlope * Value.", tjMax[i]),
                                                          new ParameterDescription("TSlope [°C]", "Temperature slope of the digital thermal sensor.\n" + "Temperature = TjMax - TSlope * Value.", 1)
                                                      },
                                                      settings);

                    ActivateSensor(_coreTemperatures[i]);
                    coreSensorId++;
                }
            }
            else
                _coreTemperatures = new Sensor[0];

            // check if processor supports a digital thermal sensor at package level
            if (cpuId[0][0].Data.GetLength(0) > 6 && (cpuId[0][0].Data[6, 0] & 0x40) != 0 && _microArchitecture != MicroArchitecture.Unknown)
            {
                _packageTemperature = new Sensor("CPU Package",
                                                 coreSensorId,
                                                 SensorType.Temperature,
                                                 this,
                                                 new[]
                                                 {
                                                     new ParameterDescription("TjMax [°C]", "TjMax temperature of the package sensor.\n" + "Temperature = TjMax - TSlope * Value.", tjMax[0]),
                                                     new ParameterDescription("TSlope [°C]", "Temperature slope of the digital thermal sensor.\n" + "Temperature = TjMax - TSlope * Value.", 1)
                                                 },
                                                 settings);

                ActivateSensor(_packageTemperature);
                coreSensorId++;
            }

            // dist to tjmax sensor
            if (cpuId[0][0].Data.GetLength(0) > 6 && (cpuId[0][0].Data[6, 0] & 1) != 0 && _microArchitecture != MicroArchitecture.Unknown)
            {
                _distToTjMaxTemperatures = new Sensor[_coreCount];
                for (int i = 0; i < _distToTjMaxTemperatures.Length; i++)
                {
                    _distToTjMaxTemperatures[i] = new Sensor(CoreString(i) + " Distance to TjMax", coreSensorId, SensorType.Temperature, this, settings);
                    ActivateSensor(_distToTjMaxTemperatures[i]);
                    coreSensorId++;
                }
            }
            else
                _distToTjMaxTemperatures = new Sensor[0];

            //core temp avg and max value
            //is only available when the cpu has more than 1 core
            if (cpuId[0][0].Data.GetLength(0) > 6 && (cpuId[0][0].Data[6, 0] & 0x40) != 0 && _microArchitecture != MicroArchitecture.Unknown && _coreCount > 1)
            {
                _coreMax = new Sensor("Core Max", coreSensorId, SensorType.Temperature, this, settings);
                ActivateSensor(_coreMax);
                coreSensorId++;

                _coreAvg = new Sensor("Core Average", coreSensorId, SensorType.Temperature, this, settings);
                ActivateSensor(_coreAvg);
            }
            else
            {
                _coreMax = null;
                _coreAvg = null;
            }

            _busClock = new Sensor("Bus Speed", 0, SensorType.Clock, this, settings);
            _coreClocks = new Sensor[_coreCount];
            for (int i = 0; i < _coreClocks.Length; i++)
            {
                _coreClocks[i] = new Sensor(CoreString(i), i + 1, SensorType.Clock, this, settings);
                if (HasTimeStampCounter && _microArchitecture != MicroArchitecture.Unknown)
                    ActivateSensor(_coreClocks[i]);
            }

            if (_microArchitecture == MicroArchitecture.SandyBridge ||
                _microArchitecture == MicroArchitecture.IvyBridge ||
                _microArchitecture == MicroArchitecture.Haswell ||
                _microArchitecture == MicroArchitecture.Broadwell ||
                _microArchitecture == MicroArchitecture.Skylake ||
                _microArchitecture == MicroArchitecture.Silvermont ||
                _microArchitecture == MicroArchitecture.Airmont ||
                _microArchitecture == MicroArchitecture.Goldmont ||
                _microArchitecture == MicroArchitecture.KabyLake ||
                _microArchitecture == MicroArchitecture.GoldmontPlus ||
                _microArchitecture == MicroArchitecture.CannonLake ||
                _microArchitecture == MicroArchitecture.IceLake ||
                _microArchitecture == MicroArchitecture.TigerLake ||
                _microArchitecture == MicroArchitecture.Tremont)
            {
                _powerSensors = new Sensor[_energyStatusMsrs.Length];
                _lastEnergyTime = new DateTime[_energyStatusMsrs.Length];
                _lastEnergyConsumed = new uint[_energyStatusMsrs.Length];

                if (Ring0.ReadMsr(MSR_RAPL_POWER_UNIT, out uint eax, out uint _))
                    switch (_microArchitecture)
                    {
                        case MicroArchitecture.Silvermont:
                        case MicroArchitecture.Airmont:
                            _energyUnitMultiplier = 1.0e-6f * (1 << (int)((eax >> 8) & 0x1F));
                            break;
                        default:
                            _energyUnitMultiplier = 1.0f / (1 << (int)((eax >> 8) & 0x1F));
                            break;
                    }

                if (_energyUnitMultiplier != 0)
                {
                    string[] powerSensorLabels = { "CPU Package", "CPU Cores", "CPU Graphics", "CPU Memory" }; 
                    
                    for (int i = 0; i < _energyStatusMsrs.Length; i++)
                    {
                        if (!Ring0.ReadMsr(_energyStatusMsrs[i], out eax, out uint _))
                            continue;


                        _lastEnergyTime[i] = DateTime.UtcNow;
                        _lastEnergyConsumed[i] = eax;
                        _powerSensors[i] = new Sensor(powerSensorLabels[i],
                                                      i,
                                                      SensorType.Power,
                                                      this,
                                                      settings);

                        ActivateSensor(_powerSensors[i]);
                    }
                }
            }

            // Enable performance counters
            if (_microArchitecture == MicroArchitecture.SandyBridge ||
                _microArchitecture == MicroArchitecture.IvyBridge ||
                _microArchitecture == MicroArchitecture.Haswell ||
                _microArchitecture == MicroArchitecture.Skylake ||
                _microArchitecture == MicroArchitecture.KabyLake)
            {
                int threadCount = 0;
                for (int coreIdx = 0; coreIdx < _coreCount; coreIdx++)
                {
                    for (int threadIdx = 0; threadIdx < cpuId[coreIdx].Length; threadIdx++)
                    {
                        threadCount++;
                    }
                }

                retiredInstructions = new Sensor[threadCount];
                unhaltedThreadClocks = new Sensor[threadCount];
                IPC = new Sensor[threadCount];
                L1CacheLoadBandwidth = new Sensor[threadCount];
                L2CacheLoadBandwidth = new Sensor[threadCount];
                L1DHitRate = new Sensor[threadCount];
                L2HitRate = new Sensor[threadCount];

                int sensorIdx = 0;
                for (int coreIdx = 0; coreIdx < _coreCount; coreIdx++)
                {
                    for (int threadIdx = 0; threadIdx < cpuId[coreIdx].Length; threadIdx++)
                    {
                        // enable fixed performance counters (3) and programmable counters (4)
                        ulong enablePMCsValue = 1 |          // enable PMC0
                                                1UL << 1 |   // enable PMC1
                                                1UL << 2 |   // enable PMC2
                                                1UL << 3 |   // enable PMC3
                                                1UL << 32 |  // enable FixedCtr0 - retired instructions
                                                1UL << 33 |  // enable FixedCtr1 - unhalted cycles
                                                1UL << 34;   // enable FixedCtr2 - reference clocks
                        Ring0.WriteMsr(IA32_PERF_GLOBAL_CTRL, enablePMCsValue, 1UL << cpuId[coreIdx][threadIdx].Thread);

                        // set all three fixed performance counters to count usr+os. Set AnyThread = false, PMI = false
                        ulong fixedCounterConfigurationValue = 1 |        // enable FixedCtr0 for os (count kernel mode instructions retired)
                                                               1UL << 1 | // enable FixedCtr0 for usr (count user mode instructions retired)
                                                               1UL << 4 | // enable FixedCtr1 for os (count kernel mode unhalted thread cycles)
                                                               1UL << 5 | // enable FixedCtr1 for usr (count user mode unhalted thread cycles)
                                                               1UL << 8 | // enable FixedCtr2 for os (reference clocks in kernel mode)
                                                               1UL << 9;  // enable FixedCtr2 for usr (reference clocks in user mode)
                        Ring0.WriteMsr(IA32_FIXED_CTR_CTRL, fixedCounterConfigurationValue, 1UL << cpuId[coreIdx][threadIdx].Thread);

                        // program PMCs. Lots of stuff we can measure. For now let's do cache hits for retired instructions
                        // Events vary between CPU models, but the events/umasks are the same between SKL/HSW for retired memory load and L2 cache events
                        if (_microArchitecture == MicroArchitecture.Skylake || 
                            _microArchitecture == MicroArchitecture.KabyLake || 
                            _microArchitecture == MicroArchitecture.Haswell)
                        {
                            // Set PMC0 to count retired memory load instructions or uops for which at least one uop hit in the L1 data cache (L1D)
                            // Haswell counts uops, Skylake counts instructions. 
                            ulong retiredL1HitLoads = GetPerfEvtSelRegisterValue(0xD1, 0x01, true, true, false, false, false, false, true, false, 0);
                            Ring0.WriteMsr(IA32_PERFEVTSEL0, retiredL1HitLoads, 1UL << cpuId[coreIdx][threadIdx].Thread);

                            // Set PMC1 to count all memory loads retired. Same difference between HSW and SKL (uops/instrs)
                            ulong retiredLoads = GetPerfEvtSelRegisterValue(0xD0, 0x81, true, true, false, false, false, false, true, false, 0);
                            Ring0.WriteMsr(IA32_PERFEVTSEL1, retiredLoads, 1UL << cpuId[coreIdx][threadIdx].Thread);

                            // Set PMC2 to count L2 misses. Speculative event, includes reads from purged instructions, prefetches, code reads, etc.
                            ulong speculativeL2Misses = GetPerfEvtSelRegisterValue(0x24, 0x3F, true, true, false, false, false, false, true, false, 0);
                            Ring0.WriteMsr(IA32_PERFEVTSEL2, speculativeL2Misses, 1UL << cpuId[coreIdx][threadIdx].Thread);

                            // Set PMC3 to count L2 requests. Also a speculative event
                            ulong speculativeL2References = GetPerfEvtSelRegisterValue(0x24, 0xFF, true, true, false, false, false, false, true, false, 0);
                            Ring0.WriteMsr(IA32_PERFEVTSEL3, speculativeL2References, 1UL << cpuId[coreIdx][threadIdx].Thread);

                            L1CacheLoadBandwidth[sensorIdx] = new Sensor(string.Format("Core {0} Thread {1} L1D Load Bandwidth", coreIdx, threadIdx),
                                                                         sensorIdx * 2,
                                                                         true,
                                                                         SensorType.Throughput,
                                                                         this,
                                                                         null,
                                                                         settings);
                            L2CacheLoadBandwidth[sensorIdx] = new Sensor(string.Format("Core {0} Thread {1} L2 Bandwidth", coreIdx, threadIdx),
                                                                         sensorIdx * 2 + 1,
                                                                         true,
                                                                         SensorType.Throughput,
                                                                         this,
                                                                         null,
                                                                         settings);
                            L1DHitRate[sensorIdx] = new Sensor(string.Format("Core {0} Thread {1} L1D Hitrate", coreIdx, threadIdx),
                                                               sensorIdx * 2,
                                                               true,
                                                               SensorType.Level,
                                                               this,
                                                               null,
                                                               settings);
                            L2HitRate[sensorIdx] = new Sensor(string.Format("Core {0} Thread {1} L2 Hitrate", coreIdx, threadIdx),
                                                              sensorIdx * 2 + 1,
                                                              true,
                                                              SensorType.Level,
                                                              this,
                                                              null,
                                                              settings);

                            ActivateSensor(L1CacheLoadBandwidth[sensorIdx]);
                            ActivateSensor(L2CacheLoadBandwidth[sensorIdx]);
                            ActivateSensor(L1DHitRate[sensorIdx]);
                            ActivateSensor(L2HitRate[sensorIdx]);
                            pmcsInitialized = true;
                        } 

                        // Zero all performance counters, so starting max values are a little less crazy
                        Ring0.WriteMsr(IA32_FIXED_CTR0, 0, 1UL << cpuId[coreIdx][threadIdx].Thread);
                        Ring0.WriteMsr(IA32_FIXED_CTR1, 0, 1UL << cpuId[coreIdx][threadIdx].Thread);
                        Ring0.WriteMsr(IA32_FIXED_CTR2, 0, 1UL << cpuId[coreIdx][threadIdx].Thread);
                        Ring0.WriteMsr(IA32_A_PMC0, 0, 1UL << cpuId[coreIdx][threadIdx].Thread);
                        Ring0.WriteMsr(IA32_A_PMC1, 0, 1UL << cpuId[coreIdx][threadIdx].Thread);
                        Ring0.WriteMsr(IA32_A_PMC2, 0, 1UL << cpuId[coreIdx][threadIdx].Thread);
                        Ring0.WriteMsr(IA32_A_PMC3, 0, 1UL << cpuId[coreIdx][threadIdx].Thread);

                        retiredInstructions[sensorIdx] = new Sensor(string.Format("Core {0} Thread {1} Instructions", coreIdx, threadIdx),
                                                                    sensorIdx,
                                                                    true,
                                                                    SensorType.Counter,
                                                                    this, 
                                                                    null,
                                                                    settings);
                        unhaltedThreadClocks[sensorIdx] = new Sensor(string.Format("Core {0} Thread {1} Active Cycles", coreIdx, threadIdx),
                                                                     sensorIdx,
                                                                     true,
                                                                     SensorType.Counter,
                                                                     this,
                                                                     null,
                                                                     settings);
                        IPC[sensorIdx] = new Sensor(string.Format("Core {0} Thread {1} IPC", coreIdx, threadIdx),
                                                    sensorIdx,
                                                    true,
                                                    SensorType.CounterRatio,
                                                    this,
                                                    null,
                                                    settings);

                        ActivateSensor(retiredInstructions[sensorIdx]);
                        ActivateSensor(unhaltedThreadClocks[sensorIdx]);
                        ActivateSensor(IPC[sensorIdx]);
                        sensorIdx++;
                    }
                }

                totalRetiredInstructions = new Sensor("Total Instructions", 0, SensorType.Counter, this, settings);
                totalUnhaltedThreadClocks = new Sensor("Total Active Thread Cycles", 0, SensorType.Counter, this, settings);
                averageIPC = new Sensor("Average IPC", 0, SensorType.CounterRatio, this, settings);

                ActivateSensor(totalRetiredInstructions);
                ActivateSensor(totalUnhaltedThreadClocks);
                ActivateSensor(averageIPC);

                if (pmcsInitialized)
                {
                    totalL1CacheLoadBandwidth = new Sensor("Total L1D Load Bandwidth", threadCount * 2, SensorType.Throughput, this, settings);
                    totalL2CacheLoadBandwidth = new Sensor("Total L2 Load Bandwidth", threadCount * 2 + 1, SensorType.Throughput, this, settings);
                    averageL1DHitRate = new Sensor("Average L1D Hitrate", threadCount * 2, SensorType.Level, this, settings);
                    averageL2HitRate = new Sensor("Average L2 Hitrate", threadCount * 2 + 1, SensorType.Level, this, settings);
                    ActivateSensor(totalL1CacheLoadBandwidth);
                    ActivateSensor(totalL2CacheLoadBandwidth);
                    ActivateSensor(averageL1DHitRate);
                    ActivateSensor(averageL2HitRate);
                }

                pmuInitialized = true;

                // Skylake Client, or Haswell Client have always-running integrated memory controller counters
                // Sandy Bridge Client does too but I don't have one to play with
                if (_model == 0x5E || _model == 0x3C)
                {
                    // "To obtain the BAR address, read the value at Bus 0; Device 0; Function 0; Offset 48H
                    // and mask with the value 0x0007FFFFF800." Okay...
                    uint barAddress = Ring0.GetPciAddress(0, 0, 0);
                    uint barLow, barHigh;
                    Ring0.ReadPciConfig(barAddress, 0x48, out barLow);
                    Ring0.ReadPciConfig(barAddress, 0x48 + 4, out barHigh); // not sure if needed, always comes out zero

                    // we get 0xfed10001 (0xfed10000 with mask), which is consistent with what others get
                    imcBar = (barLow | (ulong)barHigh << 32) & 0x0007FFFFF800;

                    // On some systems, WinRing0 fails with error 87 (invalid parameter), likely because it wasn't
                    // compiled with _PHYSICAL_MEMORY_SUPPORT defined. 
                    uint testDramDataRead = 0;
                    if (Ring0.ReadMemory<uint>(imcBar + DRAM_DATA_READS_OFFSET, ref testDramDataRead))
                    {
                        dramReadBandwidth = new Sensor("DRAM Read Bandwidth", threadCount * 2 + 2, SensorType.Throughput, this, settings);
                        dramWriteBandwidth = new Sensor("DRAM Write Bandwidth", threadCount * 2 + 3, SensorType.Throughput, this, settings);
                        dramBandwidth = new Sensor("Total DRAM Bandwidth", threadCount * 2 + 4, SensorType.Throughput, this, settings);

                        ActivateSensor(dramReadBandwidth);
                        ActivateSensor(dramWriteBandwidth);
                        ActivateSensor(dramBandwidth);
                    }
                    else
                    {
                        imcBar = 0;
                    }
                }

                if (_model == 0x5E)
                {
                    // Set up Skylake client uncore counters
                    // MSR_UNC_PERF_GLOBAL_CTRL bit 29 = enables all uncore counters. The rest deal with PMIs which we don't want
                    ulong enableSkylakeUncoreCounters = 1UL << 29;
                    Ring0.WriteMsr(SKL_MSR_UNC_PERF_GLOBAL_CTRL, enableSkylakeUncoreCounters);

                    // MSR_UNC_PERF_FIXED_CTRL bit 22 = enable counting for fixed counter
                    // Only other non-reserved bit (20) enables overflow propagation. We're gonna read this counter every second so it won't overflow
                    ulong enableSkylakeFixedCounter = 1UL << 22;
                    Ring0.WriteMsr(SKL_MSR_UNC_PERF_FIXED_CTRL, enableSkylakeFixedCounter);

                    ulong cboConfig;
                    Ring0.ReadMsr(SKL_MSR_UNC_CBO_CONFIG, out cboConfig);

                    // bits 0-3 = number of C-Box units with programmable counters
                    cboBankCount = (uint)(cboConfig & 0xF);
                    cboxL3Hits = new Sensor[cboBankCount];
                    cboxL3Lookups = new Sensor[cboBankCount];
                    cboxL3Hitrate = new Sensor[cboBankCount];

                    for(uint cboIdx = 0; cboIdx < cboBankCount; cboIdx++)
                    {
                        uint eventSelect0Msr = SKL_MSR_UNC_CBO_PERFEVTSEL0_base + (cboIdx * SKL_MSR_UNC_CBO_offset);
                        uint eventSelect1Msr = SKL_MSR_UNC_CBO_PERFEVTSEL1_base + (cboIdx * SKL_MSR_UNC_CBO_offset);
                        uint cboxCounter0Msr = SKL_MSR_UNC_CBO_PERFCTR0_base + (cboIdx * SKL_MSR_UNC_CBO_offset);
                        uint cboxCounter1Msr = SKL_MSR_UNC_CBO_PERFCTR1_base + (cboIdx * SKL_MSR_UNC_CBO_offset);

                        // counter 0: count L3 hits that don't require a line to be forwarded from another core
                        // 0x34 = cache lookup, 0x87 umask = line found in M, E, or S state.
                        // not actually listed in uncore performance monitoring doc, but 0x86 is ANY_ES and 0x81 is ANY_M so...
                        ulong event0Selection = GetCboPerfEvtSelRegisterValue(0x34, 0x86, false, false, true, false, 0);
                        Ring0.WriteMsr(eventSelect0Msr, event0Selection);
                        Ring0.WriteMsr(cboxCounter0Msr, 0);

                        // counter1: count all L3 lookups
                        // 0x34 = cache lookup, 0x8F umask = line found in MESI state (aka any state)
                        ulong event1Selection = GetCboPerfEvtSelRegisterValue(0x34, 0x8F, false, false, true, false, 0);
                        Ring0.WriteMsr(eventSelect1Msr, event1Selection);
                        Ring0.WriteMsr(cboxCounter1Msr, 0);

                        cboxL3Hits[cboIdx] = new Sensor(string.Format("CBo {0} L3 Hits", cboIdx),
                                                        (int)(threadCount + cboIdx),
                                                        true,
                                                        SensorType.Counter,
                                                        this,
                                                        null,
                                                        settings);
                        cboxL3Lookups[cboIdx] = new Sensor(string.Format("CBo {0} L3 Lookups", cboIdx),
                                                        (int)(threadCount + cboIdx),
                                                        true,
                                                        SensorType.Counter,
                                                        this,
                                                        null,
                                                        settings);
                        cboxL3Hitrate[cboIdx] = new Sensor(string.Format("CBo {0} L3 Hitrate", cboIdx),
                                                                (int)(threadCount + cboIdx),
                                                                true,
                                                                SensorType.CounterRatio,
                                                                this,
                                                                null,
                                                                settings);
                        ActivateSensor(cboxL3Hits[cboIdx]);
                        ActivateSensor(cboxL3Lookups[cboIdx]);
                        ActivateSensor(cboxL3Hitrate[cboIdx]);
                    }

                    cboxL3HitBandwidth = new Sensor("Total CBo L3 Hit Bandwidth", threadCount * 2 + 2, SensorType.Throughput, this, settings);
                    averageL3HitRate = new Sensor("Average L3 Hitrate", threadCount * 2 + 2, SensorType.Level, this, settings);
                    ActivateSensor(cboxL3HitBandwidth);
                    ActivateSensor(averageL3HitRate);
                    uncorePmuInitialized = true;
                }
            }

            Update();
        }

        private float[] Floats(float f)
        {
            float[] result = new float[_coreCount];
            for (int i = 0; i < _coreCount; i++)
                result[i] = f;

            return result;
        }

        private float[] GetTjMaxFromMsr()
        {
            float[] result = new float[_coreCount];
            for (int i = 0; i < _coreCount; i++)
            {
                if (Ring0.ReadMsr(IA32_TEMPERATURE_TARGET, out uint eax, out uint _, 1UL << _cpuId[i][0].Thread))
                    result[i] = (eax >> 16) & 0xFF;
                else
                    result[i] = 100;
            }

            return result;
        }

        protected override uint[] GetMsrs()
        {
            return new[]
            {
                MSR_PLATFORM_INFO,
                IA32_PERF_STATUS,
                IA32_THERM_STATUS_MSR,
                IA32_TEMPERATURE_TARGET,
                IA32_PACKAGE_THERM_STATUS,
                MSR_RAPL_POWER_UNIT,
                MSR_PKG_ENERY_STATUS,
                MSR_DRAM_ENERGY_STATUS,
                MSR_PP0_ENERY_STATUS,
                MSR_PP1_ENERY_STATUS
            };
        }

        public override string GetReport()
        {
            StringBuilder r = new StringBuilder();
            r.Append(base.GetReport());
            r.Append("MicroArchitecture: ");
            r.AppendLine(_microArchitecture.ToString());
            r.Append("Time Stamp Counter Multiplier: ");
            r.AppendLine(_timeStampCounterMultiplier.ToString(CultureInfo.InvariantCulture));
            r.AppendLine();
            return r.ToString();
        }

        public override void Update()
        {
            base.Update();

            float coreMax = float.MinValue;
            float coreAvg = 0;

            for (int i = 0; i < _coreTemperatures.Length; i++)
            {
                // if reading is valid
                if (Ring0.ReadMsr(IA32_THERM_STATUS_MSR, out uint eax, out uint _, 1UL << _cpuId[i][0].Thread) && (eax & 0x80000000) != 0)
                {
                    // get the dist from tjMax from bits 22:16
                    float deltaT = (eax & 0x007F0000) >> 16;
                    float tjMax = _coreTemperatures[i].Parameters[0].Value;
                    float tSlope = _coreTemperatures[i].Parameters[1].Value;
                    _coreTemperatures[i].Value = tjMax - tSlope * deltaT;

                    coreAvg += (float)_coreTemperatures[i].Value;
                    if (coreMax < _coreTemperatures[i].Value)
                        coreMax = (float)_coreTemperatures[i].Value;

                    _distToTjMaxTemperatures[i].Value = deltaT;
                }
                else
                {
                    _coreTemperatures[i].Value = null;
                    _distToTjMaxTemperatures[i].Value = null;
                }
            }

            //calculate average cpu temperature over all cores
            if (_coreMax != null && coreMax != float.MinValue)
            {
                _coreMax.Value = coreMax;
                coreAvg /= _coreTemperatures.Length;
                _coreAvg.Value = coreAvg;
            }

            if (_packageTemperature != null)
            {
                // if reading is valid
                if (Ring0.ReadMsr(IA32_PACKAGE_THERM_STATUS, out uint eax, out uint _, 1UL << _cpuId[0][0].Thread) && (eax & 0x80000000) != 0)
                {
                    // get the dist from tjMax from bits 22:16
                    float deltaT = (eax & 0x007F0000) >> 16;
                    float tjMax = _packageTemperature.Parameters[0].Value;
                    float tSlope = _packageTemperature.Parameters[1].Value;
                    _packageTemperature.Value = tjMax - tSlope * deltaT;
                }
                else
                    _packageTemperature.Value = null;
            }

            if (HasTimeStampCounter && _timeStampCounterMultiplier > 0)
            {
                double newBusClock = 0;
                for (int i = 0; i < _coreClocks.Length; i++)
                {
                    System.Threading.Thread.Sleep(1);
                    if (Ring0.ReadMsr(IA32_PERF_STATUS, out uint eax, out uint _, 1UL << _cpuId[i][0].Thread))
                    {
                        newBusClock = TimeStampCounterFrequency / _timeStampCounterMultiplier;
                        switch (_microArchitecture)
                        {
                            case MicroArchitecture.Nehalem:
                            {
                                uint multiplier = eax & 0xff;
                                _coreClocks[i].Value = (float)(multiplier * newBusClock);
                                break;
                            }

                            case MicroArchitecture.SandyBridge:
                            case MicroArchitecture.IvyBridge:
                            case MicroArchitecture.Haswell:
                            case MicroArchitecture.Broadwell:
                            case MicroArchitecture.Silvermont:
                            case MicroArchitecture.Airmont:
                            case MicroArchitecture.Skylake:
                            case MicroArchitecture.Goldmont:
                            case MicroArchitecture.KabyLake:
                            case MicroArchitecture.GoldmontPlus:
                            case MicroArchitecture.CannonLake:
                            case MicroArchitecture.IceLake:
                            case MicroArchitecture.TigerLake:
                            case MicroArchitecture.Tremont:
                            {
                                uint multiplier = (eax >> 8) & 0xff;
                                _coreClocks[i].Value = (float)(multiplier * newBusClock);
                                break;
                            }

                            default:
                            {
                                double multiplier = ((eax >> 8) & 0x1f) + 0.5 * ((eax >> 14) & 1);
                                _coreClocks[i].Value = (float)(multiplier * newBusClock);
                                break;
                            }
                        }
                    }
                    else
                    {
                        // if IA32_PERF_STATUS is not available, assume TSC frequency
                        _coreClocks[i].Value = (float)TimeStampCounterFrequency;
                    }
                }

                if (newBusClock > 0)
                {
                    _busClock.Value = (float)newBusClock;
                    ActivateSensor(_busClock);
                }
            }

            if (_powerSensors != null)
            {
                foreach (Sensor sensor in _powerSensors)
                {
                    if (sensor == null)
                        continue;

                    if (!Ring0.ReadMsr(_energyStatusMsrs[sensor.Index], out uint eax, out uint _))
                        continue;


                    DateTime time = DateTime.UtcNow;
                    uint energyConsumed = eax;
                    float deltaTime = (float)(time - _lastEnergyTime[sensor.Index]).TotalSeconds;
                    if (deltaTime < 0.01)
                        continue;


                    sensor.Value = _energyUnitMultiplier * unchecked(energyConsumed - _lastEnergyConsumed[sensor.Index]) / deltaTime;
                    _lastEnergyTime[sensor.Index] = time;
                    _lastEnergyConsumed[sensor.Index] = energyConsumed;
                }
            }

            // read performance monitoring counters
            if (pmuInitialized)
            {
                ulong totalInstructionCount = 0, totalCycleCount = 0, totalL1Hits = 0, totalL2Hits = 0, totalLoads = 0, totalL2Requests = 0;
                int sensorIdx = 0;
                for (int coreIdx = 0; coreIdx < _coreCount; coreIdx++)
                {
                    for (int threadIdx = 0; threadIdx < CpuId[coreIdx].Length; threadIdx++)
                    {
                        // read fixed counters
                        ulong threadInstructions = ReadAndClearMsr(IA32_FIXED_CTR0, 1UL << CpuId[coreIdx][threadIdx].Thread);
                        ulong threadUnhaltedClocks = ReadAndClearMsr(IA32_FIXED_CTR1, 1UL << CpuId[coreIdx][threadIdx].Thread);
                        
                        retiredInstructions[sensorIdx].Value = (float)((double)threadInstructions / 1000000000);
                        unhaltedThreadClocks[sensorIdx].Value = (float)((double)threadUnhaltedClocks / 1000000000);
                        IPC[sensorIdx].Value = (float)((double)threadInstructions / threadUnhaltedClocks);

                        totalInstructionCount += threadInstructions;
                        totalCycleCount += threadUnhaltedClocks;

                        if (pmcsInitialized)
                        {
                            ulong L1Hits = ReadAndClearMsr(IA32_A_PMC0, 1UL << CpuId[coreIdx][threadIdx].Thread);
                            ulong loadsRetired = ReadAndClearMsr(IA32_A_PMC1, 1UL << CpuId[coreIdx][threadIdx].Thread);
                            ulong L2Misses = ReadAndClearMsr(IA32_A_PMC2, 1UL << CpuId[coreIdx][threadIdx].Thread);
                            ulong L2Requests = ReadAndClearMsr(IA32_A_PMC3, 1UL << CpuId[coreIdx][threadIdx].Thread);

                            // L1D calculation only applies to SKL client, which does 32B loads from the L1 data cache
                            // SKL-X does 64B loads, so this will be wrong for those chips. Rip.
                            L1CacheLoadBandwidth[sensorIdx].Value = L1Hits * 32;
                            L2CacheLoadBandwidth[sensorIdx].Value = (L2Requests - L2Misses) * 64;
                            L1DHitRate[sensorIdx].Value = (float)(100 * (double)L1Hits / loadsRetired);
                            L2HitRate[sensorIdx].Value = (float)(100 * (1 - (double)L2Misses / L2Requests));

                            totalL1Hits += L1Hits;
                            totalL2Hits += L2Requests - L2Misses;
                            totalL2Requests += L2Requests;
                            totalLoads += loadsRetired;
                        }

                        sensorIdx++;
                    }
                }

                totalRetiredInstructions.Value = (float)((double)totalInstructionCount / 1000000000);
                totalUnhaltedThreadClocks.Value = (float)((double)totalCycleCount / 1000000000);
                averageIPC.Value = (float)((double)totalInstructionCount / totalCycleCount);

                if (pmcsInitialized)
                {
                    // same caveat as above - SKL-X does 64B loads
                    // if I get around to sandy bridge, that does 16B loads. That'll need another case
                    totalL1CacheLoadBandwidth.Value = totalL1Hits * 32;
                    totalL2CacheLoadBandwidth.Value = totalL2Hits * 64;
                    averageL1DHitRate.Value = (float)(100 * (double)totalL1Hits / totalLoads);
                    averageL2HitRate.Value = (float)(100 * (double)totalL2Hits / totalL2Requests);
                }
            }

            if (imcBar != 0)
            {
                // counters are 32 bits wide
                uint dramDataReads = 0, dramDataWrites = 0;
                Ring0.ReadMemory<uint>(imcBar + DRAM_DATA_READS_OFFSET, ref dramDataReads);
                Ring0.ReadMemory<uint>(imcBar + DRAM_DATA_WRITES_OFFSET, ref dramDataWrites);

                if (lastDramReadsValue != 0 && lastDramWritesValue != 0)
                {
                    // using uint.MaxValue to account for wrapping around seems to result in some overcounting
                    // prefer undercounting for now
                    ulong dramBytesRead = (dramDataReads > lastDramReadsValue ? (ulong)dramDataReads - lastDramReadsValue : dramDataReads) * 64;
                    ulong dramBytesWriten = (dramDataWrites > lastDramWritesValue ? (ulong)dramDataWrites - lastDramWritesValue : dramDataReads) * 64;
                    dramReadBandwidth.Value = dramBytesRead;
                    dramWriteBandwidth.Value = dramBytesWriten;
                    dramBandwidth.Value = dramBytesRead + dramBytesWriten;
                }

                lastDramReadsValue = dramDataReads;
                lastDramWritesValue = dramDataWrites;
            }

            if (uncorePmuInitialized)
            {
                ulong uncoreClocks, L3Hits = 0, L3Lookups = 0;
                Ring0.ReadMsr(SKL_MSR_UNC_PERF_FIXED_CTR, out uncoreClocks);
                Ring0.WriteMsr(SKL_MSR_UNC_PERF_FIXED_CTR, 0);

                for (uint cboIdx = 0; cboIdx < cboBankCount; cboIdx++)
                {
                    ulong cboL3HitCount, cboL3LookupCount;
                    uint counter0Msr = SKL_MSR_UNC_CBO_PERFCTR0_base + (cboIdx * SKL_MSR_UNC_CBO_offset);
                    uint counter1Msr = SKL_MSR_UNC_CBO_PERFCTR1_base + (cboIdx * SKL_MSR_UNC_CBO_offset);
                    Ring0.ReadMsr(counter0Msr, out cboL3HitCount);
                    Ring0.ReadMsr(counter1Msr, out cboL3LookupCount);
                    Ring0.WriteMsr(counter0Msr, 0);
                    Ring0.WriteMsr(counter1Msr, 0);

                    cboxL3Hits[cboIdx].Value = (float)((double)cboL3HitCount / 1000000000);
                    cboxL3Lookups[cboIdx].Value = (float)((double)cboL3LookupCount / 1000000000);
                    cboxL3Hitrate[cboIdx].Value = (float)((double)cboL3HitCount / cboL3LookupCount);

                    L3Hits += cboL3HitCount;
                    L3Lookups += cboL3LookupCount;
                }

                cboxL3HitBandwidth.Value = L3Hits * 64;
                averageL3HitRate.Value = (float)(100 * (double)L3Hits / L3Lookups);
            }
        }

        [SuppressMessage("ReSharper", "IdentifierTypo")]
        private enum MicroArchitecture
        {
            Unknown,
            NetBurst,
            Core,
            Atom,
            Nehalem,
            SandyBridge,
            IvyBridge,
            Haswell,
            Broadwell,
            Silvermont,
            Skylake,
            Airmont,
            KabyLake,
            Goldmont,
            GoldmontPlus,
            CannonLake,
            IceLake,
            TigerLake,
            Tremont
        }

        // ReSharper disable InconsistentNaming
        private const uint IA32_PACKAGE_THERM_STATUS = 0x1B1;
        private const uint IA32_PERF_STATUS = 0x0198;
        private const uint IA32_TEMPERATURE_TARGET = 0x01A2;
        private const uint IA32_THERM_STATUS_MSR = 0x019C;

        private const uint MSR_DRAM_ENERGY_STATUS = 0x619;
        private const uint MSR_PKG_ENERY_STATUS = 0x611;
        private const uint MSR_PLATFORM_INFO = 0xCE;
        private const uint MSR_PP0_ENERY_STATUS = 0x639;
        private const uint MSR_PP1_ENERY_STATUS = 0x641;

        private const uint MSR_RAPL_POWER_UNIT = 0x606;

        private const uint IA32_PERF_GLOBAL_CTRL = 0x38F;
        private const uint IA32_FIXED_CTR_CTRL = 0x38D;
        private const uint IA32_FIXED_CTR0 = 0x309;
        private const uint IA32_FIXED_CTR1 = 0x30A;
        private const uint IA32_FIXED_CTR2 = 0x30B;
        private const uint IA32_PERFEVTSEL0 = 0x186;
        private const uint IA32_PERFEVTSEL1 = 0x187;
        private const uint IA32_PERFEVTSEL2 = 0x188;
        private const uint IA32_PERFEVTSEL3 = 0x189;
        private const uint IA32_A_PMC0 = 0x4C1;
        private const uint IA32_A_PMC1 = 0x4C2;
        private const uint IA32_A_PMC2 = 0x4C3;
        private const uint IA32_A_PMC3 = 0x4C4;

        // sandy bridge and later client platform IMC counter offsets
        private const uint DRAM_GT_REQUESTS_OFFSET = 0x5040;
        private const uint DRAM_IA_REQUESTS_OFFSET = 0x5044;
        private const uint DRAM_IO_REQUESTS_OFFSET = 0x5048;
        private const uint DRAM_DATA_READS_OFFSET = 0x5050;
        private const uint DRAM_DATA_WRITES_OFFSET = 0x5054;

        // skl client uncore MSRs
        private const uint SKL_MSR_UNC_PERF_GLOBAL_CTRL = 0xE01;
        private const uint SKL_MSR_UNC_PERF_FIXED_CTRL = 0x394;
        private const uint SKL_MSR_UNC_PERF_FIXED_CTR = 0x395;
        private const uint SKL_MSR_UNC_ARB_PERFCTR0 = 0x3B0;
        private const uint SKL_MSR_UNC_ARB_PERFCTR1 = 0x3B1;
        private const uint SKL_MSR_UNC_ARB_PERFEVTSEL0 = 0x3B2;
        private const uint SKL_MSR_UNC_ARB_PERFEVTSEL1 = 0x3B3;
        private const uint SKL_MSR_UNC_CBO_CONFIG = 0x396;
        private const uint SKL_MSR_UNC_CBO_PERFEVTSEL0_base = 0x700;
        private const uint SKL_MSR_UNC_CBO_PERFEVTSEL1_base = 0x701;
        private const uint SKL_MSR_UNC_CBO_PERFCTR0_base = 0x706;
        private const uint SKL_MSR_UNC_CBO_PERFCTR1_base = 0x707;
        private const uint SKL_MSR_UNC_CBO_offset = 0x10;

        // ReSharper restore InconsistentNaming

        /// <summary>
        /// Generate value to put in IA32_PERFEVTSELx MSR
        /// for programming PMCs
        /// </summary>
        /// <param name="perfEvent">Event selection</param>
        /// <param name="umask">Umask (more specific condition for event)</param>
        /// <param name="usr">Count user mode events</param>
        /// <param name="os">Count kernel mode events</param>
        /// <param name="edge">Edge detect</param>
        /// <param name="pc">Pin control (???)</param>
        /// <param name="interrupt">Trigger interrupt on counter overflow</param>
        /// <param name="anyThread">Count across all logical processors</param>
        /// <param name="enable">Enable the counter</param>
        /// <param name="invert">Invert cmask condition</param>
        /// <param name="cmask">if not zero, count when increment >= cmask</param>
        /// <returns>Value to put in performance event select register</returns>
        private ulong GetPerfEvtSelRegisterValue(byte perfEvent, 
                                           byte umask, 
                                           bool usr, 
                                           bool os, 
                                           bool edge, 
                                           bool pc, 
                                           bool interrupt, 
                                           bool anyThread, 
                                           bool enable, 
                                           bool invert, 
                                           byte cmask)
        {
            ulong value = (ulong)perfEvent |
                (ulong)umask << 8 |
                (usr ? 1UL : 0UL) << 16 |
                (os ? 1UL : 0UL) << 17 |
                (edge ? 1UL : 0UL) << 18 |
                (pc ? 1UL : 0UL) << 19 |
                (interrupt ? 1UL : 0UL) << 20 |
                (anyThread ? 1UL : 0UL) << 21 |
                (enable ? 1UL : 0UL) << 22 |
                (invert ? 1UL : 0UL) << 23 |
                (ulong)cmask << 24;
            return value;
        }

        /// <summary>
        /// Generates value to put in uncore coherence box performance event select register
        /// </summary>
        /// <param name="perfEvent">Event selection</param>
        /// <param name="umask">Unit mask</param>
        /// <param name="edge">Count on event edge</param>
        /// <param name="interrupt">Indicate overflow via PMI</param>
        /// <param name="enable">Locally enable counter</param>
        /// <param name="invert">Invert threshold condition</param>
        /// <param name="threshold">0 = no threshold comparison. otherwise, if invert = 0, count if increment >= threshold. vice versa if invert = 1</param>
        /// <returns></returns>
        private ulong GetCboPerfEvtSelRegisterValue(byte perfEvent, 
                                                    byte umask,
                                                    bool edge,
                                                    bool interrupt,
                                                    bool enable,
                                                    bool invert,
                                                    byte threshold)
        {
            ulong value = (ulong)perfEvent |
                (ulong)umask << 8 |
                (edge ? 1UL : 0UL) << 18 |
                (interrupt ? 1UL : 0UL) << 20 |
                (enable ? 1UL : 0UL) << 22 |
                (invert ? 1UL : 0UL) << 23 |
                (ulong)threshold << 24;
            return value;
        }

        /// <summary>
        /// Read, then zero a MSR. 
        /// Useful for reading performance counters
        /// </summary>
        /// <param name="index">MSR index. Be careful</param>
        /// <param name="threadAffinityMask">thread affinity</param>
        /// <returns>MSR value</returns>
        private ulong ReadAndClearMsr(uint index, ulong threadAffinityMask)
        {
            ulong value;
            if (Ring0.ReadMsr(index, out value, threadAffinityMask))
            {
                Ring0.WriteMsr(index, 0, threadAffinityMask);
            }

            return value;
        }
    }
}
