import pandas as pd
import matplotlib.pyplot as plt
import sys

# 1. Coloca aquí el nombre exacto de tu archivo Excel (con la extensión .xlsx)
archivo_excel = 'robot_master_tracking_trial1.xlsx'  # Cambia esto por el nombre de tu archivo

try:
    # Leemos el archivo usando pandas (ahora con read_excel)
    data = pd.read_excel(archivo_excel)
except FileNotFoundError:
    print(f"❌ Error: No se encontró el archivo '{archivo_excel}'. Revisa el nombre.")
    sys.exit()
except Exception as e:
    print(f"❌ Error al abrir el Excel: {e}")
    sys.exit()

# Extraemos la columna de tiempo
tiempo = data['time_s']

# 2. Configuramos la figura: 6 filas (joints) x 2 columnas (Posición vs Error)
fig, axs = plt.subplots(6, 2, figsize=(14, 18))
fig.suptitle('Análisis de Teleoperación: Seguimiento Maestro-Esclavo (xArm 6)', fontsize=18, fontweight='bold')

for i in range(1, 7):
    row = i - 1
    
    # Extraemos los datos del motor actual
    q_m = data[f'q{i}_master_rad']
    q_s = data[f'q{i}_slave_rad']
    e = data[f'e{i}_rad']
    
    # --- COLUMNA IZQUIERDA: Posiciones (Maestro vs Esclavo) ---
    axs[row, 0].plot(tiempo, q_m, label=f'Master q{i}', color='blue', linestyle='--', linewidth=2)
    axs[row, 0].plot(tiempo, q_s, label=f'Slave q{i}', color='red', alpha=0.7, linewidth=2)
    axs[row, 0].set_ylabel(f'Joint {i} (rad)', fontweight='bold')
    axs[row, 0].legend(loc='upper right')
    axs[row, 0].grid(True, linestyle=':', alpha=0.7)
    
    # Título solo en la primera fila
    if row == 0:
        axs[row, 0].set_title('Trayectorias Articulares', fontsize=14)
        
    # --- COLUMNA DERECHA: Error de seguimiento ---
    axs[row, 1].plot(tiempo, e, label=f'Error e{i}', color='green', linewidth=1.5)
    axs[row, 1].set_ylabel('Error (rad)', fontweight='bold')
    axs[row, 1].legend(loc='upper right')
    axs[row, 1].grid(True, linestyle=':', alpha=0.7)
    
    # Título solo en la primera fila
    if row == 0:
        axs[row, 1].set_title('Error de Seguimiento (e = Master - Slave)', fontsize=14)

    # Etiqueta del eje X solo en la última fila de abajo
    if row == 5:
        axs[row, 0].set_xlabel('Tiempo (s)', fontweight='bold')
        axs[row, 1].set_xlabel('Tiempo (s)', fontweight='bold')

# Ajustamos los espacios para que no se amontone el texto
plt.tight_layout(rect=[0, 0.03, 1, 0.98])

# Mostramos la gráfica
print(f"✅ Generando gráficas para {archivo_excel}...")
plt.show()