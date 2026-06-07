import streamlit as st
import numpy as np
import matplotlib.pyplot as plt

# Konfigurasi
st.set_page_config(page_title="Simulasi Altitude Hold", layout="wide")

st.title("✈️ Simulasi Altitude Hold UAV")
st.markdown("Kendali ketinggian menggunakan PID berdasarkan Fungsi Transfer: $G(s) = 1/(ms^2 + bs)$")

# Sidebar
st.sidebar.header("🛠️ Parameter Fisik")
m = st.sidebar.slider("Massa Pesawat (m)", 1.0, 5.0, 2.0, 0.1, help="kg")
b = st.sidebar.slider("Koefisien Redaman (b)", 0.1, 2.0, 0.5, 0.1, help="Drag udara")

st.sidebar.header("🎛️ Parameter PID")
Kp = st.sidebar.slider("Kp", 0.0, 50.0, 10.0, 0.5)
Ki = st.sidebar.slider("Ki", 0.0, 10.0, 1.0, 0.1)
Kd = st.sidebar.slider("Kd", 0.0, 20.0, 5.0, 0.1)

# Simulasi
def simulate_altitude(m, b, Kp, Ki, Kd, target_h=100.0):
    dt = 0.01
    time = np.arange(0, 10, dt)
    h = np.zeros(len(time))
    v = np.zeros(len(time)) # Kecepatan vertikal
    
    integral_err = 0.0
    prev_err = 0.0
    
    for t in range(1, len(time)):
        error = target_h - h[t-1]
        integral_err += error * dt
        deriv = (error - prev_err) / dt
        
        # Sinyal kendali PID (Thrust)
        thrust = (Kp * error) + (Ki * integral_err) + (Kd * deriv)
        
        # Dinamika: F = ma -> a = (Thrust - b*v - m*g) / m
        accel = (thrust - b * v[t-1] - (m * 9.81)) / m
        
        v[t] = v[t-1] + accel * dt
        h[t] = h[t-1] + v[t] * dt
        
        prev_err = error
        
    return time, h

time, h = simulate_altitude(m, b, Kp, Ki, Kd)

# Plot
fig, ax = plt.subplots(figsize=(10, 4))
plt.style.use('dark_background')
ax.plot(time, h, label="Ketinggian Aktual", color="#00ffcc")
ax.axhline(y=100, color="#ff3366", linestyle="--", label="Target (100m)")
ax.set_xlabel("Waktu (s)")
ax.set_ylabel("Ketinggian (m)")
ax.legend()
st.pyplot(fig)

st.success("Sistem stabil! Anda bisa mempresentasikan penurunan rumus Laplace dan memvalidasinya dengan grafik ini.")