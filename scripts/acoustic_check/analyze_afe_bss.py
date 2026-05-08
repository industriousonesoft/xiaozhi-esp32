#!/usr/bin/env python3
import argparse
import array
import csv
import math
import tempfile
import wave
from pathlib import Path


EPSILON = 1e-12
CLIP_THRESHOLD = 32760
MAX_DELAY_MS = 200


def read_wav_mono(path):
    with wave.open(str(path), "rb") as wav:
        channels = wav.getnchannels()
        sample_width = wav.getsampwidth()
        sample_rate = wav.getframerate()
        frames = wav.readframes(wav.getnframes())

    if sample_width != 2:
        raise ValueError(f"{path}: expected 16-bit PCM, got sample width {sample_width}")

    pcm = array.array("h")
    pcm.frombytes(frames)
    if pcm.itemsize != 2:
        raise ValueError("this platform does not use 16-bit signed short arrays")

    samples = [float(value) for value in pcm]
    if channels > 1:
        samples = samples[0::channels]
    return samples, sample_rate


def write_wav_mono(path, samples, sample_rate):
    pcm = array.array("h", [max(-32768, min(32767, int(round(sample)))) for sample in samples])
    with wave.open(str(path), "wb") as wav:
        wav.setnchannels(1)
        wav.setsampwidth(2)
        wav.setframerate(sample_rate)
        wav.writeframes(pcm.tobytes())


def rms(samples):
    if not samples:
        return 0.0
    return math.sqrt(sum(sample * sample for sample in samples) / len(samples))


def db_ratio(numerator, denominator):
    return 20.0 * math.log10((numerator + EPSILON) / (denominator + EPSILON))


def clipping_rate(samples):
    if not samples:
        return 0.0
    return sum(1 for sample in samples if abs(sample) >= CLIP_THRESHOLD) / len(samples)


def estimate_delay_ms(reference, target, sample_rate):
    length = min(len(reference), len(target))
    if length == 0:
        return 0.0
    ref = reference[:length]
    tgt = target[:length]
    ref_mean = sum(ref) / length
    tgt_mean = sum(tgt) / length
    max_lag = min(int(sample_rate * MAX_DELAY_MS / 1000), length - 1)

    # 不依赖 numpy 时，长音频全量互相关开销很大。这里把搜索限制在关注的
    # 声学处理延迟范围内，直接计算最佳 lag。
    best_lag = 0
    best_corr = None
    for lag in range(-max_lag, max_lag + 1):
        if lag >= 0:
            ref_start = 0
            tgt_start = lag
            count = length - lag
        else:
            ref_start = -lag
            tgt_start = 0
            count = length + lag
        corr = 0.0
        for i in range(count):
            corr += (tgt[tgt_start + i] - tgt_mean) * (ref[ref_start + i] - ref_mean)
        if best_corr is None or corr > best_corr:
            best_corr = corr
            best_lag = lag
    return best_lag / sample_rate * 1000.0


def si_sdr(estimate, reference):
    length = min(len(estimate), len(reference))
    if length == 0:
        return 0.0
    est = estimate[:length]
    ref = reference[:length]
    est_mean = sum(est) / length
    ref_mean = sum(ref) / length
    est_zero = [value - est_mean for value in est]
    ref_zero = [value - ref_mean for value in ref]
    # 将估计信号投影到干净主源参考上，剩余能量视为失真/干扰。
    # 这个指标适合比较主声源保留程度。
    ref_energy = sum(value * value for value in ref_zero) + EPSILON
    scale = sum(est_zero[i] * ref_zero[i] for i in range(length)) / ref_energy
    projection = [scale * value for value in ref_zero]
    noise = [est_zero[i] - projection[i] for i in range(length)]
    projection_energy = sum(value * value for value in projection)
    noise_energy = sum(value * value for value in noise)
    return 10.0 * math.log10((projection_energy + EPSILON) / (noise_energy + EPSILON))


def analyze(raw_main_path, raw_secondary_path, afe_path, clean_main_path=None):
    raw_main, sample_rate = read_wav_mono(raw_main_path)
    raw_secondary, secondary_rate = read_wav_mono(raw_secondary_path)
    afe, afe_rate = read_wav_mono(afe_path)
    if sample_rate != secondary_rate or sample_rate != afe_rate:
        raise ValueError("all input WAV files must use the same sample rate")

    raw_main_rms = rms(raw_main)
    raw_secondary_rms = rms(raw_secondary)
    afe_rms = rms(afe)
    metrics = {
        "sample_rate": sample_rate,
        "raw_main_rms": raw_main_rms,
        "raw_secondary_rms": raw_secondary_rms,
        "afe_rms": afe_rms,
        "secondary_suppression_db": db_ratio(raw_secondary_rms, afe_rms),
        "main_retention_db": db_ratio(afe_rms, raw_main_rms),
        "afe_clipping_rate": clipping_rate(afe),
        "afe_delay_ms": estimate_delay_ms(raw_main, afe, sample_rate),
    }

    if clean_main_path is not None:
        clean_main, clean_rate = read_wav_mono(clean_main_path)
        if clean_rate != sample_rate:
            raise ValueError("clean main WAV must use the same sample rate")
        metrics["raw_main_si_sdr_db"] = si_sdr(raw_main, clean_main)
        metrics["afe_si_sdr_db"] = si_sdr(afe, clean_main)
        metrics["si_sdr_improvement_db"] = metrics["afe_si_sdr_db"] - metrics["raw_main_si_sdr_db"]

    return metrics


def write_report(metrics, report_path, csv_path=None):
    lines = [
        "# AFE BSS Analysis Report",
        "",
        "| Metric | Value |",
        "| --- | ---: |",
    ]
    for key, value in metrics.items():
        if isinstance(value, float):
            lines.append(f"| {key} | {value:.6f} |")
        else:
            lines.append(f"| {key} | {value} |")
    lines.append("")
    Path(report_path).write_text("\n".join(lines), encoding="utf-8")

    if csv_path is not None:
        with Path(csv_path).open("w", newline="", encoding="utf-8") as fp:
            writer = csv.writer(fp)
            writer.writerow(["metric", "value"])
            for key, value in metrics.items():
                writer.writerow([key, value])


def run_self_test():
    sample_rate = 16000
    clean = []
    interference = []
    for index in range(sample_rate):
        t = index / sample_rate
        clean.append(12000.0 * math.sin(2.0 * math.pi * 440.0 * t))
        interference.append(6000.0 * math.sin(2.0 * math.pi * 880.0 * t))
    raw_main = [clean[i] + interference[i] for i in range(sample_rate)]
    raw_secondary = interference
    afe = [clean[i] + 0.2 * interference[i] for i in range(sample_rate)]

    with tempfile.TemporaryDirectory() as tmp:
        tmp_path = Path(tmp)
        raw_main_path = tmp_path / "raw_main.wav"
        raw_secondary_path = tmp_path / "raw_secondary.wav"
        afe_path = tmp_path / "afe.wav"
        clean_path = tmp_path / "clean.wav"
        write_wav_mono(raw_main_path, raw_main, sample_rate)
        write_wav_mono(raw_secondary_path, raw_secondary, sample_rate)
        write_wav_mono(afe_path, afe, sample_rate)
        write_wav_mono(clean_path, clean, sample_rate)
        metrics = analyze(raw_main_path, raw_secondary_path, afe_path, clean_path)

    assert metrics["secondary_suppression_db"] > -7.0
    assert metrics["main_retention_db"] > -1.5
    assert metrics["si_sdr_improvement_db"] > 10.0
    assert metrics["afe_clipping_rate"] == 0.0
    print("self-test passed")


def main():
    parser = argparse.ArgumentParser(description="Analyze Korvo-1 dual-mic AFE BSS WAV captures.")
    parser.add_argument("--raw-main", type=Path, help="Raw Mic0 WAV")
    parser.add_argument("--raw-secondary", type=Path, help="Raw Mic1 WAV")
    parser.add_argument("--afe", type=Path, help="AFE output mono WAV")
    parser.add_argument("--clean-main", type=Path, help="Optional clean main-source reference WAV")
    parser.add_argument("--out", type=Path, default=Path("afe_bss_report.md"), help="Markdown report path")
    parser.add_argument("--csv", type=Path, help="Optional CSV metrics path")
    parser.add_argument("--self-test", action="store_true", help="Run synthetic signal self-test")
    args = parser.parse_args()

    if args.self_test:
        run_self_test()
        return

    required = [args.raw_main, args.raw_secondary, args.afe]
    if any(path is None for path in required):
        parser.error("--raw-main, --raw-secondary, and --afe are required unless --self-test is used")

    metrics = analyze(args.raw_main, args.raw_secondary, args.afe, args.clean_main)
    write_report(metrics, args.out, args.csv)
    print(f"wrote {args.out}")
    if args.csv is not None:
        print(f"wrote {args.csv}")


if __name__ == "__main__":
    main()
