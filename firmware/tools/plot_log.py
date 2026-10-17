#!/usr/bin/env python3
"""Plota o CSV de 1 ms gravado da serial do firmware.

Gravação (a placa envia a 921600 baud):

    stty -F /dev/ttyUSB0 921600 raw -echo
    cat /dev/ttyUSB0 > log.csv            # Ctrl+C para parar

Para o cabeçalho sair no arquivo, inicie o cat antes de dar reset na placa.

Uso:

    python3 tools/plot_log.py log.csv
"""

import sys

import matplotlib.pyplot as plt
import pandas as pd

COLUMNS = ["t_us", "pwm_a", "pwm_b", "pwm_c", "setor", "pos", "pos_unwrap", "vel_rpm_x10"]


def load(path: str) -> pd.DataFrame:
    # Linhas com '#' são os logs de texto; o cabeçalho pode faltar se o cat
    # começou depois do reset, então os nomes são passados explicitamente.
    df = pd.read_csv(path, comment="#", header=None, names=COLUMNS,
                     on_bad_lines="skip", dtype=str)
    # Com o cabeçalho no arquivo, usa só o que veio depois do último (descarta o
    # resto de uma execução anterior ao reset)
    header_rows = df.index[df["t_us"] == "t_us"]
    if len(header_rows) > 0:
        df = df.loc[header_rows[-1] + 1:]

    df = df.apply(pd.to_numeric, errors="coerce").dropna()  # linha cortada no início/fim
    df = df.reset_index(drop=True)

    df["t_s"] = (df["t_us"] - df["t_us"].iloc[0]) / 1e6
    df["vel_rpm"] = df["vel_rpm_x10"] / 10.0
    return df


def main() -> None:
    if len(sys.argv) != 2:
        sys.exit(f"uso: {sys.argv[0]} log.csv")

    df = load(sys.argv[1])

    dt = df["t_us"].diff().dropna()
    lost = int((dt > 1500).sum())
    print(f"{len(df)} linhas, {df['t_s'].iloc[-1]:.3f} s, "
          f"intervalo médio {dt.mean():.0f} us, {lost} salto(s) por linhas descartadas")

    fig, ax = plt.subplots(5, 1, sharex=True, figsize=(11, 10))

    for c in ("pwm_a", "pwm_b", "pwm_c"):
        ax[0].plot(df["t_s"], df[c], label=c)
    ax[0].set_ylabel("PWM (CCR)")
    ax[0].legend(loc="upper right")

    ax[1].step(df["t_s"], df["setor"], where="post")
    ax[1].set_ylabel("setor")
    ax[1].set_yticks(range(6))

    ax[2].plot(df["t_s"], df["pos"])
    ax[2].set_ylabel("pos (0..4095)")

    ax[3].plot(df["t_s"], df["pos_unwrap"])
    ax[3].set_ylabel("pos sem wrap")

    ax[4].plot(df["t_s"], df["vel_rpm"])
    ax[4].set_ylabel("velocidade (rpm)")
    ax[4].set_xlabel("tempo (s)")

    for a in ax:
        a.grid(True, alpha=0.3)

    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
