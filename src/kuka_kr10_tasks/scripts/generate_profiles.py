#!/usr/bin/env python3

"""
Generación automática de perfiles cúbico y quíntico.

Este script calcula:

1. Distancia cartesiana entre una posición inicial y una final.
2. Duración mínima del perfil cúbico.
3. Duración mínima del perfil quíntico.
4. Posición cartesiana de los waypoints.
5. Velocidad cartesiana.
6. Aceleración cartesiana.
7. Archivos CSV.
8. Gráficas de posición, velocidad y aceleración.

El script se utiliza para demostrar y documentar los perfiles exigidos
en el taller.

El nodo C++ cartesian_path realiza internamente estos mismos cálculos
durante la ejecución del robot, por lo que no depende de los CSV.

Ejemplo para pre_pick -> pick:

python3 scripts/profiles.py \
    --start 0.473678 0.466123 0.363086 \
    --goal 0.619970 0.466132 0.345948 \
    --max-velocity 0.200 \
    --max-acceleration 0.300 \
    --intermediate-points 4 \
    --output-directory profiles_pick

Ejemplo para pre_place -> place:

python3 scripts/profiles.py \
    --start 0.396007 -0.202068 0.372297 \
    --goal 0.619950 -0.202012 0.345982 \
    --max-velocity 0.200 \
    --max-acceleration 0.300 \
    --intermediate-points 4 \
    --output-directory profiles_place
"""

import argparse
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


# =============================================================================
# PERFIL CÚBICO
# =============================================================================

def cubic_profile(tau):
    """
    Perfil cúbico normalizado.

    Posición:
        s(tau) = 3*tau² - 2*tau³

    Velocidad normalizada:
        ds/dtau = 6*tau - 6*tau²

    Aceleración normalizada:
        d²s/dtau² = 6 - 12*tau

    Condiciones:

        s(0) = 0
        s(1) = 1

        ds/dtau(0) = 0
        ds/dtau(1) = 0

    La aceleración no es cero en los extremos.
    """

    s = (
        3.0 * tau**2
        - 2.0 * tau**3
    )

    ds_dtau = (
        6.0 * tau
        - 6.0 * tau**2
    )

    d2s_dtau2 = (
        6.0
        - 12.0 * tau
    )

    return s, ds_dtau, d2s_dtau2


# =============================================================================
# PERFIL QUÍNTICO
# =============================================================================

def quintic_profile(tau):
    """
    Perfil quíntico normalizado.

    Posición:
        s(tau) = 10*tau³ - 15*tau⁴ + 6*tau⁵

    Velocidad normalizada:
        ds/dtau = 30*tau² - 60*tau³ + 30*tau⁴

    Aceleración normalizada:
        d²s/dtau² = 60*tau - 180*tau² + 120*tau³

    Condiciones:

        s(0) = 0
        s(1) = 1

        ds/dtau(0) = 0
        ds/dtau(1) = 0

        d²s/dtau²(0) = 0
        d²s/dtau²(1) = 0
    """

    s = (
        10.0 * tau**3
        - 15.0 * tau**4
        + 6.0 * tau**5
    )

    ds_dtau = (
        30.0 * tau**2
        - 60.0 * tau**3
        + 30.0 * tau**4
    )

    d2s_dtau2 = (
        60.0 * tau
        - 180.0 * tau**2
        + 120.0 * tau**3
    )

    return s, ds_dtau, d2s_dtau2


# =============================================================================
# DISTANCIA CARTESIANA
# =============================================================================

def calculate_cartesian_length(start_position, goal_position):
    """
    Calcula:

        delta_p = goal - start

        length = ||delta_p||
    """

    displacement = (
        goal_position
        - start_position
    )

    length = np.linalg.norm(
        displacement
    )

    if length <= 0.0:
        raise ValueError(
            "La distancia cartesiana debe ser mayor que cero."
        )

    direction = (
        displacement
        / length
    )

    return displacement, length, direction


# =============================================================================
# DURACIÓN DEL PERFIL CÚBICO
# =============================================================================

def calculate_cubic_duration(
    length,
    maximum_velocity,
    maximum_acceleration,
):
    """
    Duración requerida por velocidad:

        Tv = 1.5 * L / vmax

    Duración requerida por aceleración:

        Ta = sqrt(6 * L / amax)

    Duración final:

        T = max(Tv, Ta)
    """

    velocity_time = (
        1.5
        * length
        / maximum_velocity
    )

    acceleration_time = math.sqrt(
        6.0
        * length
        / maximum_acceleration
    )

    duration = max(
        velocity_time,
        acceleration_time,
    )

    return (
        duration,
        velocity_time,
        acceleration_time,
    )


# =============================================================================
# DURACIÓN DEL PERFIL QUÍNTICO
# =============================================================================

def calculate_quintic_duration(
    length,
    maximum_velocity,
    maximum_acceleration,
):
    """
    Máxima velocidad normalizada:

        max(ds/dtau) = 1.875

    Máxima aceleración normalizada:

        max(|d²s/dtau²|) = 10 / sqrt(3)

    Duración por velocidad:

        Tv = 1.875 * L / vmax

    Duración por aceleración:

        Ta = sqrt(
            (10/sqrt(3)) * L / amax
        )
    """

    maximum_normalized_velocity = (
        1.875
    )

    maximum_normalized_acceleration = (
        10.0
        / math.sqrt(3.0)
    )

    velocity_time = (
        maximum_normalized_velocity
        * length
        / maximum_velocity
    )

    acceleration_time = math.sqrt(
        maximum_normalized_acceleration
        * length
        / maximum_acceleration
    )

    duration = max(
        velocity_time,
        acceleration_time,
    )

    return (
        duration,
        velocity_time,
        acceleration_time,
    )


# =============================================================================
# GENERACIÓN DE MUESTRAS
# =============================================================================

def generate_profile_samples(
    profile_function,
    start_position,
    goal_position,
    duration,
    intermediate_points,
):
    """
    Genera:

        1 punto inicial
        3 o 4 puntos intermedios
        1 punto final

    Para cada punto calcula:

        t
        tau
        s
        posición XYZ
        velocidad XYZ
        aceleración XYZ
        magnitud de velocidad
        magnitud de aceleración
    """

    displacement = (
        goal_position
        - start_position
    )

    length = np.linalg.norm(
        displacement
    )

    total_points = (
        intermediate_points
        + 2
    )

    samples = []

    for index in range(total_points):

        tau = (
            index
            / (total_points - 1)
        )

        time = (
            tau
            * duration
        )

        (
            s,
            ds_dtau,
            d2s_dtau2,
        ) = profile_function(tau)

        # Conversión de derivadas respecto a tau
        # a derivadas respecto al tiempo.

        ds_dt = (
            ds_dtau
            / duration
        )

        d2s_dt2 = (
            d2s_dtau2
            / duration**2
        )

        # Posición cartesiana.

        position = (
            start_position
            + s * displacement
        )

        # Velocidad cartesiana.

        velocity = (
            ds_dt
            * displacement
        )

        # Aceleración cartesiana.

        acceleration = (
            d2s_dt2
            * displacement
        )

        velocity_magnitude = np.linalg.norm(
            velocity
        )

        acceleration_magnitude = np.linalg.norm(
            acceleration
        )

        signed_acceleration = (
            length
            * d2s_dt2
        )

        samples.append(
            {
                "index": index,
                "t": time,
                "tau": tau,
                "s": s,
                "distance": length * s,
                "x": position[0],
                "y": position[1],
                "z": position[2],
                "vx": velocity[0],
                "vy": velocity[1],
                "vz": velocity[2],
                "velocity": velocity_magnitude,
                "ax": acceleration[0],
                "ay": acceleration[1],
                "az": acceleration[2],
                "acceleration": acceleration_magnitude,
                "signed_acceleration": signed_acceleration,
            }
        )

    return samples


# =============================================================================
# PERFIL CONTINUO PARA GRÁFICAS
# =============================================================================

def generate_continuous_profile(
    profile_function,
    length,
    duration,
    number_of_samples=501,
):
    """
    Genera una versión densa del perfil para:

        gráficas
        máximos continuos aproximados
    """

    tau_values = np.linspace(
        0.0,
        1.0,
        number_of_samples,
    )

    time_values = (
        tau_values
        * duration
    )

    s_values = np.zeros_like(
        tau_values
    )

    velocity_values = np.zeros_like(
        tau_values
    )

    acceleration_values = np.zeros_like(
        tau_values
    )

    for index, tau in enumerate(tau_values):

        (
            s,
            ds_dtau,
            d2s_dtau2,
        ) = profile_function(tau)

        s_values[index] = s

        velocity_values[index] = (
            length
            * ds_dtau
            / duration
        )

        acceleration_values[index] = (
            length
            * d2s_dtau2
            / duration**2
        )

    return {
        "t": time_values,
        "tau": tau_values,
        "s": s_values,
        "distance": length * s_values,
        "velocity": velocity_values,
        "acceleration": acceleration_values,
    }


# =============================================================================
# ESCRITURA DE CSV
# =============================================================================

def write_csv(file_path, samples):
    """
    Guarda todos los datos de los waypoints.
    """

    field_names = [
        "index",
        "t",
        "tau",
        "s",
        "distance",
        "x",
        "y",
        "z",
        "vx",
        "vy",
        "vz",
        "velocity",
        "ax",
        "ay",
        "az",
        "acceleration",
        "signed_acceleration",
    ]

    with file_path.open(
        mode="w",
        newline="",
        encoding="utf-8",
    ) as csv_file:

        writer = csv.DictWriter(
            csv_file,
            fieldnames=field_names,
        )

        writer.writeheader()

        for sample in samples:

            row = {}

            for key in field_names:

                if key == "index":
                    row[key] = sample[key]
                else:
                    row[key] = (
                        f"{sample[key]:.12f}"
                    )

            writer.writerow(row)


# =============================================================================
# IMPRESIÓN DE RESULTADOS
# =============================================================================

def print_profile_summary(
    profile_name,
    duration,
    velocity_time,
    acceleration_time,
    samples,
    continuous_profile,
    maximum_velocity,
    maximum_acceleration,
):
    """
    Muestra tiempos, máximos y waypoints.
    """

    continuous_maximum_velocity = (
        np.max(
            np.abs(
                continuous_profile["velocity"]
            )
        )
    )

    continuous_maximum_acceleration = (
        np.max(
            np.abs(
                continuous_profile["acceleration"]
            )
        )
    )

    velocity_condition = (
        continuous_maximum_velocity
        <= maximum_velocity + 1e-9
    )

    acceleration_condition = (
        continuous_maximum_acceleration
        <= maximum_acceleration + 1e-9
    )

    print()
    print("=" * 78)
    print(f"PERFIL {profile_name.upper()}")
    print("=" * 78)

    print(
        f"Duración final:             "
        f"{duration:.6f} s"
    )

    print(
        f"Duración por velocidad:     "
        f"{velocity_time:.6f} s"
    )

    print(
        f"Duración por aceleración:   "
        f"{acceleration_time:.6f} s"
    )

    print(
        f"Velocidad máxima continua:  "
        f"{continuous_maximum_velocity:.6f} m/s"
    )

    print(
        f"Límite de velocidad:        "
        f"{maximum_velocity:.6f} m/s"
    )

    print(
        f"Aceleración máxima continua:"
        f" {continuous_maximum_acceleration:.6f} m/s²"
    )

    print(
        f"Límite de aceleración:      "
        f"{maximum_acceleration:.6f} m/s²"
    )

    print(
        f"Cumple velocidad:           "
        f"{'SÍ' if velocity_condition else 'NO'}"
    )

    print(
        f"Cumple aceleración:         "
        f"{'SÍ' if acceleration_condition else 'NO'}"
    )

    print()
    print(
        " idx     t [s]      tau         s"
        "         x [m]       y [m]       z [m]"
        "      |v| [m/s]    a_signed [m/s²]"
    )

    for sample in samples:

        print(
            f"{sample['index']:4d}  "
            f"{sample['t']:10.6f}  "
            f"{sample['tau']:8.6f}  "
            f"{sample['s']:8.6f}  "
            f"{sample['x']:11.6f}  "
            f"{sample['y']:11.6f}  "
            f"{sample['z']:11.6f}  "
            f"{sample['velocity']:12.6f}  "
            f"{sample['signed_acceleration']:16.6f}"
        )


# =============================================================================
# GRÁFICAS
# =============================================================================

def create_comparison_plot(
    cubic_continuous,
    quintic_continuous,
    output_file,
):
    """
    Genera una figura con:

        distancia
        velocidad
        aceleración
    """

    figure, axes = plt.subplots(
        3,
        1,
        figsize=(10, 12),
    )

    # Posición o distancia recorrida.

    axes[0].plot(
        cubic_continuous["t"],
        cubic_continuous["distance"],
        label="Cúbico",
    )

    axes[0].plot(
        quintic_continuous["t"],
        quintic_continuous["distance"],
        label="Quíntico",
    )

    axes[0].set_title(
        "Posición cartesiana sobre el recorrido"
    )

    axes[0].set_xlabel(
        "Tiempo [s]"
    )

    axes[0].set_ylabel(
        "Distancia recorrida [m]"
    )

    axes[0].grid(True)

    axes[0].legend()

    # Velocidad.

    axes[1].plot(
        cubic_continuous["t"],
        cubic_continuous["velocity"],
        label="Cúbico",
    )

    axes[1].plot(
        quintic_continuous["t"],
        quintic_continuous["velocity"],
        label="Quíntico",
    )

    axes[1].set_title(
        "Velocidad cartesiana"
    )

    axes[1].set_xlabel(
        "Tiempo [s]"
    )

    axes[1].set_ylabel(
        "Velocidad [m/s]"
    )

    axes[1].grid(True)

    axes[1].legend()

    # Aceleración.

    axes[2].plot(
        cubic_continuous["t"],
        cubic_continuous["acceleration"],
        label="Cúbico",
    )

    axes[2].plot(
        quintic_continuous["t"],
        quintic_continuous["acceleration"],
        label="Quíntico",
    )

    axes[2].set_title(
        "Aceleración cartesiana"
    )

    axes[2].set_xlabel(
        "Tiempo [s]"
    )

    axes[2].set_ylabel(
        "Aceleración [m/s²]"
    )

    axes[2].grid(True)

    axes[2].legend()

    figure.tight_layout()

    figure.savefig(
        output_file,
        dpi=200,
    )

    plt.close(figure)


# =============================================================================
# ARGUMENTOS
# =============================================================================

def parse_arguments():
    """
    Lee los argumentos de terminal.
    """

    parser = argparse.ArgumentParser(
        description=(
            "Genera perfiles cúbico y quíntico "
            "entre dos posiciones cartesianas."
        )
    )

    parser.add_argument(
        "--start",
        type=float,
        nargs=3,
        required=True,
        metavar=("X0", "Y0", "Z0"),
        help=(
            "Posición cartesiana inicial en metros."
        ),
    )

    parser.add_argument(
        "--goal",
        type=float,
        nargs=3,
        required=True,
        metavar=("XF", "YF", "ZF"),
        help=(
            "Posición cartesiana final en metros."
        ),
    )

    parser.add_argument(
        "--max-velocity",
        type=float,
        default=0.200,
        help=(
            "Velocidad cartesiana máxima en m/s. "
            "Valor predeterminado: 0.200."
        ),
    )

    parser.add_argument(
        "--max-acceleration",
        type=float,
        default=0.300,
        help=(
            "Aceleración cartesiana máxima en m/s². "
            "Valor predeterminado: 0.300."
        ),
    )

    parser.add_argument(
        "--intermediate-points",
        type=int,
        choices=[3, 4],
        default=4,
        help=(
            "Cantidad de puntos intermedios. "
            "Debe ser 3 o 4."
        ),
    )

    parser.add_argument(
        "--output-directory",
        type=Path,
        default=Path("profiles"),
        help=(
            "Directorio de salida."
        ),
    )

    return parser.parse_args()


# =============================================================================
# MAIN
# =============================================================================

def main():
    """
    Función principal.
    """

    arguments = parse_arguments()

    if arguments.max_velocity <= 0.0:
        raise ValueError(
            "La velocidad máxima debe ser positiva."
        )

    if arguments.max_acceleration <= 0.0:
        raise ValueError(
            "La aceleración máxima debe ser positiva."
        )

    start_position = np.array(
        arguments.start,
        dtype=float,
    )

    goal_position = np.array(
        arguments.goal,
        dtype=float,
    )

    (
        displacement,
        length,
        direction,
    ) = calculate_cartesian_length(
        start_position,
        goal_position,
    )

    (
        cubic_duration,
        cubic_velocity_time,
        cubic_acceleration_time,
    ) = calculate_cubic_duration(
        length,
        arguments.max_velocity,
        arguments.max_acceleration,
    )

    (
        quintic_duration,
        quintic_velocity_time,
        quintic_acceleration_time,
    ) = calculate_quintic_duration(
        length,
        arguments.max_velocity,
        arguments.max_acceleration,
    )

    cubic_samples = generate_profile_samples(
        cubic_profile,
        start_position,
        goal_position,
        cubic_duration,
        arguments.intermediate_points,
    )

    quintic_samples = generate_profile_samples(
        quintic_profile,
        start_position,
        goal_position,
        quintic_duration,
        arguments.intermediate_points,
    )

    cubic_continuous = generate_continuous_profile(
        cubic_profile,
        length,
        cubic_duration,
    )

    quintic_continuous = generate_continuous_profile(
        quintic_profile,
        length,
        quintic_duration,
    )

    arguments.output_directory.mkdir(
        parents=True,
        exist_ok=True,
    )

    cubic_file = (
        arguments.output_directory
        / "cubic.csv"
    )

    quintic_file = (
        arguments.output_directory
        / "quintic.csv"
    )

    plot_file = (
        arguments.output_directory
        / "profiles_comparison.png"
    )

    write_csv(
        cubic_file,
        cubic_samples,
    )

    write_csv(
        quintic_file,
        quintic_samples,
    )

    create_comparison_plot(
        cubic_continuous,
        quintic_continuous,
        plot_file,
    )

    print()
    print("=" * 78)
    print("DATOS DEL MOVIMIENTO")
    print("=" * 78)

    print(
        "Posición inicial: "
        f"[{start_position[0]:.6f}, "
        f"{start_position[1]:.6f}, "
        f"{start_position[2]:.6f}] m"
    )

    print(
        "Posición final:   "
        f"[{goal_position[0]:.6f}, "
        f"{goal_position[1]:.6f}, "
        f"{goal_position[2]:.6f}] m"
    )

    print(
        "Desplazamiento:   "
        f"[{displacement[0]:.6f}, "
        f"{displacement[1]:.6f}, "
        f"{displacement[2]:.6f}] m"
    )

    print(
        f"Longitud:          "
        f"{length:.9f} m"
    )

    print(
        "Dirección:        "
        f"[{direction[0]:.6f}, "
        f"{direction[1]:.6f}, "
        f"{direction[2]:.6f}]"
    )

    print(
        f"Puntos intermedios:"
        f" {arguments.intermediate_points}"
    )

    print(
        f"Velocidad máxima:  "
        f"{arguments.max_velocity:.6f} m/s"
    )

    print(
        f"Aceleración máxima:"
        f" {arguments.max_acceleration:.6f} m/s²"
    )

    print_profile_summary(
        "Cúbico",
        cubic_duration,
        cubic_velocity_time,
        cubic_acceleration_time,
        cubic_samples,
        cubic_continuous,
        arguments.max_velocity,
        arguments.max_acceleration,
    )

    print_profile_summary(
        "Quíntico",
        quintic_duration,
        quintic_velocity_time,
        quintic_acceleration_time,
        quintic_samples,
        quintic_continuous,
        arguments.max_velocity,
        arguments.max_acceleration,
    )

    print()
    print("=" * 78)
    print("ARCHIVOS GENERADOS")
    print("=" * 78)

    print(
        f"CSV cúbico:     {cubic_file}"
    )

    print(
        f"CSV quíntico:   {quintic_file}"
    )

    print(
        f"Gráfica:        {plot_file}"
    )


if __name__ == "__main__":
    main()