import numpy as np
import roboticstoolbox as rtb
import spatialmath as sm
import matplotlib.pyplot as plt
from roboticstoolbox import DHRobot
from scipy.linalg import null_space
from spatialmath import SE3

class DHRobotGT(DHRobot):
    """
    Extensión de la clase de roboticstoolbox con generador de trayectorias joint y cartesiano.
    """
    def __init__(self, *args, tacc=0.1, Ts=1E-3,vmax=None, **kwargs):
        """
        Args:
            tacc (float): Tiempo de aceleración para el perfil trapezoidal.
            Ts (float): Periodo de muestreo.
            vmax (np.ndarray): Vector de velocidades máximas por articulación.
        """
        super().__init__(*args, **kwargs)
        self.tacc = tacc
        self.Ts = Ts
        
        # Si no se especifica, se fija una velocidad máxima de 150 deg/s para 6 ejes
        if vmax is None:
            self.vmax = np.radians(150) * np.ones(self.n)
        else:
            self.vmax = vmax

        # Almacenamiento de trayectorias generadas
        self.t_ref = None
        self.q_ref = None
        self.qd_ref = None
        self.qdd_ref = None
        self.tau = None

    def interpoladorTrapezoidal(self,A,B,C,Tj, return_s=False, plot=False):
        """
        Interpolador trapezoidal implementando las zonas 1 (aceleración constante) y 2 (velocidad constante) 
        del método de Paul.

        Args:
          A (np.array): Punto anterior (o actual al inicio).
          B (np.array): Punto actual (pivote del movimiento).
          C (np.array): Punto siguiente (objetivo).
          Tj (float): Tiempo total estimado para el segmento.
          return_s (bool): Si True, devuelve el perfil escalar "s" normalizado (0 a 1).
          plot (bool): Si True, grafica los perfiles normalizados.

        Returns:
          q, qd, qdd: Vectores interpolados de posición, velocidad y aceleración.
          s_profile (opcional): Perfil de interpolación normalizado.
        """
        DA = A-B
        DC = C-B

        # Zona 1
        # Generar el vector tseg desde -tacc hasta +tacc
        t_z1 = np.arange(-self.tacc + self.Ts, self.tacc + self.Ts, self.Ts)

        # Cálculo de las referencias para zona 1: aceleración constante
        qdd_z1 = np.outer((DC/Tj+DA/self.tacc)/(2*self.tacc),np.ones(len(t_z1)))
        qd_z1 = (DC / Tj)[:, np.newaxis] * (t_z1 + self.tacc) / (2 * self.tacc) + (DA / self.tacc)[:, np.newaxis] * (t_z1 - self.tacc) / (2 * self.tacc)
        q_z1 = (DC / Tj)[:, np.newaxis] * (t_z1 + self.tacc)**2 / (4 * self.tacc) + (DA / self.tacc)[:, np.newaxis] * (t_z1 - self.tacc)**2 / (4 * self.tacc) + np.outer(B,np.ones(len(t_z1)))

        # Zona 2
        # Generar el vector tseg desde tacc hasta (Tj - tacc)
        t_z2 = np.arange(self.tacc + self.Ts, Tj - self.tacc + 0.5* self.Ts , self.Ts)

        # Cálculo de las referencias para zona 2: velocidad constante
        qdd_z2 = np.zeros((len(B), len(t_z2)))
        qd_z2 = np.outer(DC / Tj, np.ones(len(t_z2)))
        q_z2 = np.outer(DC / Tj, t_z2) + np.outer(B, np.ones(len(t_z2)))

        # Concatenación de zonas
        q_aux = np.hstack([q_z1, q_z2])
        qd_aux = np.hstack([qd_z1, qd_z2])
        qdd_aux = np.hstack([qdd_z1, qdd_z2])

        # qdd_z2 = np.hstack([qdd_aux,np.zeros((len(B), len(tseg)))])
        # qd_z2 = np.hstack([qd_aux,np.outer(DC / Tj, np.ones(len(tseg)))])
        # q_z2 = np.hstack([q_aux,np.outer(DC / Tj, tseg) +  np.outer(B,np.ones(len(tseg)))])

        # Diferencia total en cada coordenada
        deltas = q_aux[:,-1] - q_aux[:,0]
        # Buscar el índice del eje con mayor desplazamiento absoluto
        idx = np.argmax(np.abs(deltas))
        # Normalizar con respecto a dicho eje. Útil para graficar
        delta_max = deltas[idx]

        if np.isclose(delta_max, 0.0):
            print("No hay ninguna coordenada con desplazamiento para normalizar.")
            # Si no hay desplazamiento se utiliza un perfil trivial
            s_profile = np.array([[0,1]])
        else:
            # Perfil del interpolador
            s_profile = (q_aux[idx,:] - q_aux[idx,0]) / delta_max

        qd_max = np.max(np.abs(qd_aux[idx,:]))
        qdd_max = np.max(np.abs(qdd_aux[idx,:]))

        qd_norm = qd_aux[idx,:] / qd_max if qd_max > 0 else np.zeros_like(qd_aux[idx,:])
        qdd_norm = qdd_aux[idx,:] / qdd_max if qdd_max > 0 else np.zeros_like(qdd_aux[idx,:])

        # Obtención de gráficos normalizados
        if plot:
            self._plot_perfiles_normalizados(s_profile, qd_norm, qdd_norm)

        if return_s:
            return s_profile, qd_norm, qdd_norm, s_profile
        else:
            return q_aux, qd_aux, qdd_aux

    def genTrJoint(self, q_dest,Td):
        """
        Genera la trayectoria joint para un conjunto de puntos de paso.

        Args:
            q_dest : Matriz con los puntos de paso. Cada fila corresponde a un punto.
            Td : tiempos deseados de cada movimiento.

        Returns:
            t_ref : Vector de tiempo de referencia.
            q_ref : Vector de posiciones articulares de referencia.
            qd_ref : Vector de velocidades articulares de referencia.
            qdd_ref : Vector de aceleraciones articulares de referencia.
            POSES : Vector de posiciones cartesianas de referencia.
        """
        # Inicializar vectores
        q = np.empty((self.nlinks,0)); qd = np.empty((self.nlinks,0)); qdd = np.empty((self.nlinks,0))
        A = q_dest[0,:];

        # Identificación de puntos A, B y C
        for i in range(len(q_dest)-1):
          B = q_dest[i,:]

          if i < len(q_dest) - 1:
            C = q_dest[i + 1, :]
          else:
            C = B
            Td[i] = 0
          
          # El Tj será el máximo entre lo permitido por los motores, 2*tacc y lo solicitado
          Tj = np.max((np.max(np.abs(C - B) / self.vmax), Td[i], 2 * self.tacc))

          # Interpolación joint
          q_aux, qd_aux, qdd_aux = self.interpoladorTrapezoidal(A,B,C,Tj)
          q = np.hstack([q,q_aux]); qd = np.hstack([qd,qd_aux]); qdd = np.hstack([qdd,qdd_aux]);

          # Actualizar A para la siguiente iteración
          A = q[:, -1]

        t = np.linspace(0, q.shape[1], num = q.shape[1])*self.Ts
        

        self.t_ref = t; self.q_ref=q.T; self.qd_ref=qd.T; self.qdd_ref=qdd.T
        self.tau = np.zeros_like(self.q_ref)

        return self.t_ref, self.q_ref, self.qd_ref, self.qdd_ref

    def genTrCart(self,POSE_dest,Td, conf = [1, 1, 1], tool = SE3(), plot = False):
    
        """
        Genera la trayectoria cartesiana para un conjunto de puntos de paso. Utiliza el perfil del interpolador trapezoidal
        y SLERP para las rotaciones.

        Args:
          POSE_dest : Lista con las POSES de paso.
          Td : tiempos deseados de cada movimiento.
          conf : Configuración articular para la cinemática inversa.
          tool : Transformación del TCP con respecto a la brida.

        Returns:
          t_ref : Vector de tiempo de referencia.
          q_ref : Vector de posiciones articulares de referencia.
          qd_ref : Vector de velocidades articulares de referencia.
          qdd_ref : Vector de aceleraciones articulares de referencia.
          POSES : Vector de posiciones cartesianas de referencia.
        """
        # Inicializar variables
        n_tramos = len(POSE_dest) - 1
        POSEA = POSE_dest[0]

        POSES = []
        s_profiles = []

        # Variables para graficar
        plot_data = {'q': [], 'qd': [], 'qdd': []}
        
        # Identificación de A, B y C
        for i in range(n_tramos):
            POSEB = POSE_dest[i]
            if i < len(POSE_dest) - 1:
                POSEC = POSE_dest[i + 1]
            else:
                POSEC = POSEB
                Td[i] = 0

            Tj = np.max([Td[i], 2 * self.tacc])

            # Interpolación trapezoidal de las poses
            POSEA_q = self.ikine(POSEA*tool.inv(), conf)[0]
            POSEB_q = self.ikine(POSEB*tool.inv(), conf)[0]
            POSEC_q = self.ikine(POSEC*tool.inv(), conf)[0]
            s_prof, qd_aux, qdd_aux, _ = self.interpoladorTrapezoidal(POSEA_q, POSEB_q, POSEC_q, Tj, return_s=True)

            #Convertir el s local al tiempo global
            scale = 1.0 / n_tramos
            offset = i / n_tramos
            s_scaled = s_prof * scale + offset

            s_profiles.append(s_scaled)
            
            # Guardar datos para graficar
            if plot:
                plot_data['q'].append(s_scaled)
                plot_data['qd'].append(qd_aux)
                plot_data['qdd'].append(qdd_aux)

        s_global_full = np.concatenate(s_profiles)
        s_global_full = s_global_full.round(3)  # Redondeo para evitar problemas numéricos

        # SLERP con perfil trapezoidal
        POSES = POSEA.interp(POSEB, s=s_global_full)

        # Obtener variables articulares y luego velocidades y aceleraciones diferenciando
        q = np.zeros((len(POSES), self.nlinks))
        for i in range(len(POSES)):
            q[i,:], _ = self.ikine(POSES[i]*tool.inv(), conf)

        qd = np.diff(q, axis=0) / self.Ts
        qd = np.vstack([qd, np.zeros(self.nlinks,)])
        qdd = np.diff(qd, axis=0) / self.Ts
        qdd = np.vstack([qdd, np.zeros(self.nlinks,)])

        t = np.linspace(0, len(q), num=len(q)) * self.Ts
        self.t_ref = t; self.q_ref = q; self.qd_ref = qd; self.qdd_ref = qdd

        self.tau = np.zeros_like(self.q_ref)

        # Plotear perfiles
        if plot:
            self._plot_cartesian_profiles(
                np.concatenate(plot_data['q']),
                np.concatenate(plot_data['qd']),
                np.concatenate(plot_data['qdd'])
            )

        return self.t_ref, self.q_ref, self.qd_ref, self.qdd_ref
    
    def _plot_cartesian_profiles(self, q, qd, qdd):
        """Helper para graficar perfiles cartesianos."""
        fig, axes = plt.subplots(3, 1, sharex=True, figsize=(8, 8))
        
        labels = [r'$q$', r'$\dot{q}$', r'$\ddot{q}$']
        colors = ['#d62728', '#1f77b4', '#2ca02c']

        data = [q, qd, qdd]

        for ax, dat, lbl, col in zip(axes, data, labels, colors):
            ax.plot(dat, color=col, linewidth=2)
            ax.set_ylabel(lbl, fontsize=12)
            ax.grid(True, alpha=0.5)
        axes[0].set_title('Perfiles de posiciones, velocidades y aceleraciones', fontsize=16)

        axes[-1].set_xlabel('t', fontsize=12)
        plt.tight_layout()
        plt.show()
    
    def _plot_perfiles_normalizados(self, q, qd, qdd):
        """Helper interno para graficar perfiles de interpolador."""
        # Lógica de normalización para visualización

        plt.figure(figsize=(8, 5))
        plt.plot(q, label='q (posición)', linewidth=2)
        plt.plot(qd, label='qd (velocidad)', linewidth=2)
        plt.plot(qdd, label='qdd (aceleración)', linewidth=2)
        plt.xlabel('Step')
        plt.ylabel('Normalizado')
        plt.legend()
        plt.grid(True)
        plt.title('Perfiles Trapezoidales Normalizados')
        plt.show()

class myCobot320(DHRobotGT):
    """
    Definición del Cobot 320 Pi. Incluye métodos como la cinemática inversa, la enseñanza de TCPs, workobjects y calculo de configuraciones.
    """
    def __init__(self,*args, metros=False, rotar_base=False, **kwargs):
       
        # Definición de los enlaces usando parámetros DH. Por defecto en mm
        esc = 1e-3 if metros else 1 
        eje1 = rtb.RevoluteDH(alpha=-np.pi/2,a=0,d=173.87*esc,offset=0,qlim=[-168*np.pi/180,168*np.pi/180])
        eje2 = rtb.RevoluteDH(alpha=0,a=134.96*esc,d=0,offset=-np.pi/2,qlim=[-135*np.pi/180,135*np.pi/180])
        eje3 = rtb.RevoluteDH(alpha=0,a=119.99*esc,d=0,offset=0,qlim=[-150*np.pi/180,150*np.pi/180]) 
        eje4 = rtb.RevoluteDH(alpha=np.pi/2,a=0,d=88.74*esc,offset=np.pi/2,qlim=[-145*np.pi/180,145*np.pi/180])
        eje5 = rtb.RevoluteDH(alpha=-np.pi/2,a=0,d=94.99*esc,offset=0,qlim=[-165*np.pi/180,165*np.pi/180])    
        eje6 = rtb.RevoluteDH(alpha=0,a=0,d=65.48*esc,offset=0,qlim=[-180*np.pi/180,180*np.pi/180])
    
        # Crear la estructura del robot
        super().__init__(*args,[eje1, eje2, eje3, eje4, eje5, eje6], name='myCobot320',gravity = np.array([0, 0, -9.8]),**kwargs)
        
        # Rotar la base para que coincida con pyMyCobot
        self.rotar_base = rotar_base
        if self.rotar_base:
            self.base = sm.SE3.Rz(np.pi)

    def ikine(self, POSE, conf=np.array([1,1,1]), offset = True, graficar_alcance = False):
        """
        Cinemática Inversa analítica del robot myCobot320.

        Args:
            POSE : Pose de destino.
            conf : Vector de configuraciones (hombro, brazo, muñeca).

        Returns:
            q : Vector de variables articulares

        Raises:
            IKineError : Si la pose no es alcanzable o hay un error en la cinemática inversa.
        """
        conf1, conf2, conf3 = conf

        if len(POSE) == 7:   # Permitir al metodo funcionar con una pose extraida de fkine y de fkine_all
            pose_aux = POSE[6]
        else:
            pose_aux = POSE

        # Corrección por rotar la base
        if self.rotar_base:
            pose_aux = sm.SE3.Rz(np.pi) * POSE

        # Extraer las componentes de la matriz que van a ser usadas en las ecuaciones
        px, py, pz = pose_aux.t
        nx, ny, nz = pose_aux.R[:, 0]   
        sx, sy, sz = pose_aux.R[:, 1]
        ax, ay, az = pose_aux.R[:, 2]  


        # Leer las longitudes de eslabones y articulares de la tabla DH
        d1 = self.links[0].d
        a2 = self.links[1].a
        a3 = self.links[2].a
        d4 = self.links[3].d 
        d5 = self.links[4].d 
        d6 = self.links[5].d 

        # Theta1: indeterminación del brazo
        discr = (px - ax*d6)**2 + (py - ay*d6)**2 - d4**2

        # Identificación en caso de falla
        if discr<0:
            raise IKineError(
            "Discriminante negativo. La pose está fuera del círculo de alcance."
            )
        
        q1 = np.arctan2(d4, conf1*np.sqrt(discr)) - np.arctan2(py - ay*d6, ax*d6 - px)
        # q1 = -2*np.arctan((-ax*d6 + px + conf1*np.sqrt(discr))/(-ay*d6 + d4 + py))    # Otra expresión

        # Theta5: indeterminación de la muñeca
        q5 = np.arctan2(conf3*np.sqrt((ny*np.cos(q1) - nx*np.sin(q1))**2 + (sy*np.cos(q1) - sx*np.sin(q1))**2), ay*np.cos(q1) - ax*np.sin(q1))
        
        # Theta6: singularidad si sin(q5)=0
        q6 = np.arctan2(-conf3*(sy*np.cos(q1) - sx*np.sin(q1)), conf3*(ny*np.cos(q1) - nx*np.sin(q1)))
        # q6 = np.arctan2((sx*np.sin(q1) - sy*np.cos(q1))/np.sin(q5), (ny*np.cos(q1) - nx*np.sin(q1))/np.sin(q5))   # Otra expresión
        
        # Calcular theta2+theta3+theta4
        theta234 = np.arctan2(az*conf3, -conf3*(ax*np.cos(q1) + ay*np.sin(q1)))
        # theta234 = np.arctan2(az/np.sin(q5), -(ax*np.cos(q1) + ay*np.sin(q1))/np.sin(q5)) # Otra expresión

        # Cálculos auxiliares: sustituciones A y B
        A = px*np.cos(q1) - d5*np.sin(theta234) + py*np.sin(q1) + d6*np.sin(q5)*np.cos(theta234)
        B = d1 - pz + d5*np.cos(theta234) + d6*np.sin(q5)*np.sin(theta234)

        # Calcular c3 
        c3 = (A**2 + B**2 - a2**2 - a3**2)/(2*a2*a3)

        # Chequear que el punto sea alcanzable
        if np.abs(c3) > 1:
            raise IKineError("Alcanzabilidad. c3 fuera de [-1,1], punto no alcanzable.")
        
        # Theta3: indeterminación del codo 
        q3 = np.arctan2(conf2*np.sqrt(1 - c3**2), c3)
        
        # Calcular el seno y el coseno de theta2
        s2 = (B*a2 + B*a3*np.cos(q3) - A*a3*np.sin(q3))/(a2**2 + a3**2 + 2*a2*a3*np.cos(q3))
        c2 = (A*a2 + A*a3*np.cos(q3) + B*a3*np.sin(q3))/(a2**2 + a3**2 + 2*a2*a3*np.cos(q3))
        
        # Calcular theta2
        q2 = np.arctan2(s2, c2)
        
        # Calcular theta4
        q4 = theta234 - q2 - q3
        
        # Armar vector completo
        q = np.array([q1,q2,q3,q4,q5,q6])
        # Corregir por offset
        if offset:
            q = q - self.offset
        
        # Limitar q entre -pi y pi
        q = (q + np.pi) % (2 * np.pi) - np.pi
        status=1

        # Gráfico del modelo del robot defubuebdi el alcance (zonas permitidas) para una pose dada
        if graficar_alcance:
            # Radios máximos y mínimos
            r_min = abs(a2 - a3)
            r_max = a2 + a3

            posiciones = [np.array([0, 0, 0])]
            for i in range(7):
                posiciones.append(POSE.A[i][:3, 3])  # Posición X, Y, Z

            pos=np.array(posiciones).T
            fig = plt.figure(figsize=(10,10))
            ax3d = fig.add_subplot(111, projection='3d')

            # Posición inicial
            ax3d.plot(pos[0], pos[1], pos[2], 'o', color='red', label='Posición', zorder=3, markersize=8)

            # Unir los puntos intermedios (no necesariamente son los eslabones)
            for i in range(len(pos[0]) - 1):
                ax3d.plot(
                    [pos[0, i], pos[0, i + 1]],
                    [pos[1, i], pos[1, i + 1]],
                    [pos[2, i], pos[2, i + 1]],
                    color='black', linewidth=3
                )

            # Calcular alcances
            muñeca = np.array([px - ax * d6,
                    py - ay * d6,
                    pz - az * d6])
            theta = np.linspace(0, 2 * np.pi, 100)
            x_circ = d4 * np.cos(theta)
            y_circ = d4 * np.sin(theta)
            centro_z = muñeca[2] 
            z_circ = np.full_like(theta, centro_z)

            x_max = r_max * np.cos(theta)
            y_max = r_max * np.sin(theta)

            x_min = r_min * np.cos(theta)
            y_min = r_min * np.sin(theta)

            # Plotear alcances
            ax3d.plot(x_circ, y_circ, z_circ,
                    color='blue',
                    linestyle='-.',
                    linewidth=2,
                    label='Límite de alcance (discriminante)')
            ax3d.plot(x_max, y_max, d1, 'b--', linewidth=1, label="Alcance Máximo (c3)")
            ax3d.plot(x_min, y_min, d1,  'g--', linewidth=1, label="Alcance Mínimo (c3)")

            # Configuración del gráfico
            ax3d.set_xlim([-0.300, 0.300])
            ax3d.set_ylim([-0.300, 0.300])
            ax3d.set_zlim([0.000, 0.400])
            ax3d.set_xlabel('X (mm)', fontsize=10)
            ax3d.set_ylabel('Y (mm)', fontsize=10)
            ax3d.set_zlabel('Z (mm)', fontsize=10)
            plt.title('myCobot320: Posición de los Eslabones', fontsize=16)
            plt.legend(fontsize=11)
            plt.grid(True)
            plt.show()
            
        return q,status
    
    def calc_conf(self,q):
        """
        Calcula la configuracion a partir del vector de variables articulares.

        Args:
            q : Vector de variables articulares.

        Returns:
            Lista con la configuración del robot. Ej: [1, -1, 1].

        Nota:
            - p16_x: componente x de la transformacion que expresa la posición y orientacion de la brida
              en el sist de ref del primer eslabon. Su signo indica si el hombro está hacia adelante o hacia atras.
            - El angulo de q3 define conf2: codo arriba o abajo.
            - El angulo de q5 define conf3: muñeca arriba o abajo.
        """
        q_in = np.array(q)
                
        # Si es 1D (shape=(7,)), se convierte a (1, 7).
        if q_in.ndim == 1:
            q_mat = q_in[np.newaxis, :]
        else:
            q_mat = q_in
        
        # Si tiene 7 columnas (gripper), se corta a 6
        n_cols = q_mat.shape[1]
        if n_cols == 7:
            q_arm = q_mat[:, :6]
        elif n_cols == 6:
            q_arm = q_mat
        else:
            raise ValueError(f"Dimensión incorrecta: {n_cols}. Se esperan 6 o 7 columnas.")

        result = []
        # Calcular la pose con el PCD, extraer variables necesarias y definir configuración
        for q_row in q_arm:
            A = self.fkine_all(q_row)
            A1_6 = A[1].inv() * A[6]
            p16_x = A1_6.t[0]

            conf1 = -np.sign(p16_x)
            conf2 = np.sign(q_row[2])
            conf3 = np.sign(q_row[4])

            result.append([conf1, conf2, conf3])

        conf = np.array(result, dtype=int)

        return conf[0].tolist() if conf.shape[0] == 1 else conf.tolist()

    def graficar_conf(self, q, conf, limits=np.array([-0.100, 0.300, -0.300, 0.300, 0.000, 0.400])):
    
        """
        Grafica una pose en una configuracion dada.
        Se captura el entorno generado por la funcion plot de la libreria para modificarle los ejes.
        Tambien se guarda como png con un nombre que permite identificarla por configuracion.

        Args:
            q : Vector de variables articulares.
            conf : Lista con la configuración del robot. Ej: [1, -1, 1].
            limits : Limites de los ejes en el grafico.
        """
        # Graficar
        a = self.plot(q, backend='pyplot', limits= limits  , jointaxes = False ,block=False, name=False)
        plt.close()

        # Modificar vista
        fig = a.fig
        fig.set_size_inches(6,6)
        ax = fig.gca()
        ax.view_init(elev=41, azim=-37)
        ax.set_xlim([limits[0], limits[1]])
        ax.set_ylim([limits[2], limits[3]])
        ax.set_zlim([limits[4], limits[5]])
        ax.set_box_aspect([1, 1, 0.5]) 
        a.close()

        # Guardado
        fig.savefig(f'Imágenes\Configuraciones\\[{conf[0]}][{conf[1]}][{conf[2]}].png')
        return fig
    
    def traj_nucleo(self, q, puntos, paso, norm):
        """
        Genera una trayectoria en la direccion del nucleo del jacobiano. 
        Facilita la identificación de singularidades internas y externas.

        Args:
            q : Posicion articular inicial.
            puntos : Cantidad de puntos a generar.
            paso : Tamaño del paso a dar en cada iteracion.
            norm : Indice del eje a normalizar (0 a 5).
        """
        trayectoria_total = []
        q_actual = q

        for i in range(puntos):
            # Calcular el jacobiano y el nulo en cada punto
            J_actual = self.jacob0(q_actual)
            ns = null_space(J_actual)
            if ns.size == 0:                                  # Validar si sigue existiendo el nucleo
                print(f"En la iteracion {i} el jacobiano recupera rango. Es decir: la singularidad es externa!")
                return 0
            
            # Determinar la dirección y generar el punto siguiente con el generador joint y el paso
            direccion = ns[:, 0]
            direccion = direccion / direccion[norm]*-1
            q_siguiente = q_actual + paso * direccion
            tr_segmento = self.genTrJoint(np.array([q_actual, q_siguiente, q_siguiente]), 0*np.ones(3))
            traj_segmento = self.q_ref[::300]                 # Subsampling para aumentar velocidad
            trayectoria_total.append(traj_segmento[1:])       # Acumular la trayectoria total
            q_actual = q_siguiente                            # Actualizar el q actual
        print("Trayectoria generada con exito. La singularidad es interna!")
        return np.vstack(trayectoria_total)
 
    def TCP_4puntos(self, q_pose, z_aux = 25):
        """
        Calcula la traslación de la brida al TCP mediante el método de los 4 puntos.

        Args:
            q_pose: Poses enseñadas como vectores de variables articulares.
            z_aux: Distancia de la punta del palpador de la pinza auxiliar al TCP objetivo.

        Returns:
            p_tool: Offset TCP calculado (sin corrección).
            p_tool_real: Offset TCP corregido.
            residuals: Residuos de la solución.
            ecm: Error cuadrático medio.
            rmse: Raíz del error cuadrático medio.
        """
        # Convertir vectores de variables articulares a poses (matrices homogéneas) con el PCD
        T_list = []
        for q in q_pose:
            T_se3 = self.fkine(np.radians(q))
            T = T_se3.A
            T_list.append(T)

        # Armar sistema de ecuaciones
        A_blocks = []
        b_blocks = []
        for T in T_list:
            R = T[:3, :3]
            t = -1*T[:3, 3].reshape((3, 1))
            A_block = np.hstack([R, np.eye(3)])
            A_blocks.append(A_block)
            b_blocks.append(t)

        A = np.vstack(A_blocks)   # 12x6
        b = np.vstack(b_blocks).flatten()  # 12x1

        # Resolver el sistema sobredeterminado por cuadrados mínimos
        x, residuals, _, _ = np.linalg.lstsq(A, b, rcond=None)
        if residuals.size > 0:
            N = A.shape[0]
            ecm = residuals[0] / N
        else:
            ecm = 0.0
        rmse = np.sqrt(ecm)

        x = x.flatten()
        p_tool = x[:3]

        # Corrección por pieza auxiliar
        correccion = np.array([0, -abs(z_aux), 0])  # La extensión corresponde al eje Y de la brida
        p_tool_real = p_tool[:3] + correccion

        return p_tool, p_tool_real, residuals, ecm, rmse

    def calc_wobj(self, q_poses_deg, tool: SE3, z_aux = 25):
        """
        Calcula un workobject a partir de una lista de vectores de variables articulares.
        Permite métodos de cálculo enseñando 3 o 6 puntos.
        
        Args:
            q_poses_deg: Lista de variables articulares en grados correspondientes a las poses enseñadas.
            La cantidad de puntos define el método a usar.
            tool (SE3): TCP utilizado en la enseñanza de poses (SE3).
            z_aux: Altura de la pieza auxiliar.
        
        Returns:
            wobj (SE3): Workobject calculado.

        Raises:
            ValueError: Si el número de puntos no es adecuado para el método seleccionado.
        """
        n = len(q_poses_deg)
        
        # Preprocesamiento común: variables articulares a puntos en el espacio
        puntos = np.zeros((n, 3))
        for i in range(n):
            q_rad = np.radians(q_poses_deg[i])
            # Trasladar a la punta del palpador y extraer posición
            pose_final = self.fkine(q_rad) * tool * SE3(0, 0, abs(z_aux))
            puntos[i] = pose_final.t

        print(f'[DEBUG] Puntos calculados ({n}):\n{puntos}')

        # Definir método de cálculo y delegar al helper
        if n == 3:
            return self._compute_wobj_3points(puntos)
        elif n >= 6:
            return self._compute_wobj_6points_pca(puntos)
        else:
            raise ValueError(f"Método 'auto' no sabe qué hacer con {n} puntos. Use 3 o >=6.")        
    
    def _compute_wobj_3points(self, puntos):
        """
        Método geométrico exacto con 3 puntos (P1, P2, P3).
        """
        p1, p2, p3 = puntos[:3]

        # Eje X: de p2 a p1
        x_axis = p1 - p2
        norm_x = np.linalg.norm(x_axis)
        if norm_x < 1e-9: raise ValueError("P1 y P2 están superpuestos.")
        x_axis /= norm_x

        # Origen de la terna (Proyección de p3 sobre la recta definida por p1-p2)
        origen = p2 + np.dot(p3 - p2, x_axis) * x_axis

        # Eje Y: vector desde origen hasta p3
        v = p3 - origen
        norm_v = np.linalg.norm(v)
        if norm_v < 1e-9: raise ValueError("P3 es colineal con P1-P2.")
        y_axis = v / norm_v

        # Eje Z
        z_axis = np.cross(x_axis, y_axis)

        # Armar matriz y ortonormalizar (DVS para mejorar precisión numérica)
        wobj = self._build_se3_from_axes(x_axis, y_axis, z_axis, origen)
        
        return wobj
    
    def _compute_wobj_6points_pca(self, puntos):
        """Método estadístico (PCA) con 6 puntos (3 para X, 3 para Y)."""
        # Los primeros 3 puntos definen al eje X, los otros 3 a Y
        puntos_x = puntos[:3]
        puntos_y = puntos[3:]

        # Armar rectas con el helper
        centroid_x, x_axis, dists_x = self.fit_line_pca(puntos_x)
        centroid_y, y_axis_pca, dists_y = self.fit_line_pca(puntos_y)

        # Eje Y: proyección ortogonal sobre X
        y_axis_raw = y_axis_pca - np.dot(y_axis_pca, x_axis) * x_axis
        if np.linalg.norm(y_axis_raw) < 1e-9:
            raise ValueError("Los puntos para el eje Y son casi colineales con X.")
        y_axis = y_axis_raw / np.linalg.norm(y_axis_raw)

        # Origen: proyectar centroid_y sobre linea X
        origen = centroid_x + np.dot(centroid_y - centroid_x, x_axis) * x_axis

        # Eje Z: producto vectorial
        z_axis = np.cross(x_axis, y_axis)
        z_axis /= np.linalg.norm(z_axis)

        # Chequeo de calidad de los puntos enseñados
        stats = {
            "resid_x_mean": float(np.mean(dists_x)),
            "resid_x_std": float(np.std(dists_x)),
            "resid_y_mean": float(np.mean(dists_y)),
            "resid_y_std": float(np.std(dists_y)),
        }

        print(f"Wobj enseñado con 6 puntos. resid X mean/std: {stats['resid_x_mean']:.3f}/{stats['resid_x_std']:.3f} mm; "
            f"resid Y mean/std: {stats['resid_y_mean']:.3f}/{stats['resid_y_std']:.3f} mm")

        # Armar terna con los ejes y el origen
        wobj = self._build_se3_from_axes(x_axis, y_axis, z_axis, origen)

        return wobj
    
    def _build_se3_from_axes(self, x_axis, y_axis, z_axis, origen):
        """
        Helper para construir y ortonormalizar una terna a partir de 3 ejes y un origen.
        """
        # Rotación ortonormalizada mediante DVS
        R = np.column_stack((x_axis, y_axis, z_axis))
        U, _, Vt = np.linalg.svd(R)
        R_orth = U @ Vt
        if np.linalg.det(R_orth) < 0:
            U[:, -1] *= -1
            R_orth = U @ Vt
            
        # Debug error
        err = np.linalg.norm(R_orth.T @ R_orth - np.eye(3), 'fro')
        print(f"[Math] Orthogonality Error: {err:.2e}")

        wobj_SE3 = SE3.Rt(R_orth, origen)

        def rot_error(R):
            I = np.eye(3)
            err_orth = np.linalg.norm(R.T @ R - I, 'fro')
            err_det = abs(np.linalg.det(R) - 1)
            return err_orth, err_det

        err_orth, err_det = rot_error(R)
        print(f"Error ortogonalidad: {err_orth:.2e}, Error determinante: {err_det:.2e}")
        
        return wobj_SE3
    
    @staticmethod
    def fit_line_pca(points):
        """
        Ajuste de recta por PCA (Total Least Squares) a partir de 3 puntos en el espacio.

        Args:
            points : Posiciones en el espacio dadas por sus coordenadas x, y, z.
        
        Returns:
            centroid : Centroide geométrico del conjunto de puntos.
            direction : Dirección de la recta que mejor ajusta el conjunto de puntos.
            dists : Residuos. Distancias perpendiculares de los puntos a la recta calculada.
        """
        centroid = np.mean(points, axis=0)
        X = points - centroid

        # DVS
        U, S, Vt = np.linalg.svd(X, full_matrices=False)
        direction = Vt[0, :]
        direction = direction / np.linalg.norm(direction)

        # Distancias perpendiculares (residuales)
        proj = X @ direction[:, None] @ direction[None, :]
        perp = X - proj
        dists = np.linalg.norm(perp, axis=1)

        return centroid, direction, dists
    
class IKineError(Exception):
    """Excepción lanzada por ikine cuando no se puede calcular la solución."""
    def __init__(self, mensaje: str):
        super().__init__(mensaje)

    def __repr__(self):
        return f"IKineError(error={self.args[0]})"