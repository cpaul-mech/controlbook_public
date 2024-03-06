import hummingbirdParam as P
import tkinter as tk

class HB_GUI:
    def __init__(self,master,
                km_slider = False,
                encoder_val = False,
                lonPID = False,
                ):   

        if km_slider:
            self.km_pack(master)

        if lonPID:
            pass
            self.lonPID_pack(master)

        if encoder_val:
            self.pack_encoder_disp(master)

    # ----------------Km Slider Functions--------------------
    def km_pack(self,master):    
        km_frame = tk.LabelFrame(master, text="km", padx=10, pady=10)
        km_frame.pack(padx=10, pady=10)

        # Create and pack a label for the slider
        km_label = tk.Label(km_frame, text="km:")
        km_label.pack(side=tk.TOP)

        # Create a "Minus" button to decrease the slider value
        km_minus_button = tk.Button(km_frame, text="-", command=self.km_decrease)
        km_minus_button.pack(side=tk.LEFT)

        # Create a slider
        self.km_slider = tk.Scale(km_frame, length=200, from_=P.km_min, to=P.km_max, resolution=P.km_step, orient='horizontal')
        self.km_slider.set(P.km)
        self.km_slider.pack(side=tk.LEFT)

        # Create a "Plus" button to increase the slider value
        km_plus_button = tk.Button(km_frame, text="+", command=self.km_increase)
        km_plus_button.pack(side=tk.LEFT)

    def km_decrease(self):
        current_value = self.km_slider.get()
        new_value = max(current_value - P.km_step, P.km_min)
        self.km_slider.set(new_value)

    def km_increase(self):
        current_value = self.km_slider.get()
        new_value = min(current_value + P.km_step, P.km_max)
        self.km_slider.set(new_value)

    # ----------------lonPID Slider Functions--------------------
    def lonPID_pack(self, master):
        lonPID_frame = tk.LabelFrame(master, text="lonPID", padx=10, pady=10)
        lonPID_frame.pack(padx=10, pady=10)

        self.theta_ref_slider = self.create_slider_control(lonPID_frame, "Theta Ref:", self.theta_decrease, self.theta_increase, P.theta_ref_min, P.theta_ref_max, P.theta_ref_step, P.theta_ref)
        self.theta_kp_slider = self.create_slider_control(lonPID_frame, "Theta Kp:", self.theta_kp_decrease, self.theta_kp_increase, P.theta_kp_min, P.theta_kp_max, P.theta_kp_step, P.theta_kp)
        self.theta_kd_slider = self.create_slider_control(lonPID_frame, "Theta Kd:", self.theta_kd_decrease, self.theta_kd_increase, P.theta_kd_min, P.theta_kd_max, P.theta_kd_step, P.theta_kd)
        self.theta_ki_slider = self.create_slider_control(lonPID_frame, "Theta Ki:", self.theta_ki_decrease, self.theta_ki_increase, P.theta_ki_min, P.theta_ki_max, P.theta_ki_step, P.theta_ki)

    def create_slider_control(self, master, label_text, command_decrease, command_increase, from_, to_, resolution, start):
        frame = tk.Frame(master)
        frame.pack(side=tk.TOP, fill=tk.X, expand=True)

        label = tk.Label(frame, text=label_text)
        label.pack(side=tk.TOP)

        minus_button = tk.Button(frame, text="-", command=command_decrease)
        minus_button.pack(side=tk.LEFT)

        slider = tk.Scale(frame, length=200, from_=from_, to=to_, resolution=resolution, orient='horizontal')
        slider.set(start)
        slider.pack(side=tk.LEFT)

        plus_button = tk.Button(frame, text="+", command=command_increase)
        plus_button.pack(side=tk.LEFT)

        return slider        

    def theta_decrease(self):
        current_value = self.theta_ref_slider.get()
        new_value = max(current_value - P.theta_ref_step, P.theta_ref_min)
        self.theta_ref_slider.set(new_value)

    def theta_increase(self):
        current_value = self.theta_ref_slider.get()
        new_value = min(current_value + P.theta_ref_step, P.theta_ref_max)
        self.theta_ref_slider.set(new_value)

    def theta_kp_decrease(self):
        current_value = self.theta_kp_slider.get()
        new_value = max(current_value - P.theta_kp_step, P.theta_kp_min)
        self.theta_kp_slider.set(new_value)

    def theta_kp_increase(self):
        current_value = self.theta_kp_slider.get()
        new_value = min(current_value + P.theta_kp_step, P.theta_kp_max)
        self.theta_kp_slider.set(new_value)

    def theta_kd_decrease(self):
        current_value = self.theta_kd_slider.get()
        new_value = max(current_value - P.theta_kd_step, P.theta_kd_min)
        self.theta_kd_slider.set(new_value)

    def theta_kd_increase(self):
        current_value = self.theta_kd_slider.get()
        new_value = min(current_value + P.theta_kd_step, P.theta_kd_max)
        self.theta_kd_slider.set(new_value)

    def theta_ki_decrease(self):
        current_value = self.theta_ki_slider.get()
        new_value = max(current_value - P.theta_ki_step, P.theta_ki_min)
        self.theta_ki_slider.set(new_value)

    def theta_ki_increase(self):
        current_value = self.theta_ki_slider.get()
        new_value = min(current_value + P.theta_ki_step, P.theta_ki_max)
        self.theta_ki_slider.set(new_value)

    # ----------------Encoder Display Functions--------------------
    def pack_encoder_disp(self,master):        
        self.phi_val = tk.StringVar(master, value='0')
        self.theta_val = tk.StringVar(master, value='0')
        self.psi_val = tk.StringVar(master, value='0')

        labels = []
        values = []
        labels.append("Phi: ")
        values.append(self.phi_val)
        labels.append("Theta: ")
        values.append(self.theta_val)
        labels.append("Psi: ")
        values.append(self.psi_val)

        # A frame for the button
        Angle_frame = tk.LabelFrame(master, text="Encoder Values", padx=10, pady=10)
        Angle_frame.pack(padx=10, pady=10)

        # Use grid to place labels and their values
        for i, (label_text, var) in enumerate(zip(labels, values)):
            label = tk.Label(Angle_frame, text=label_text, anchor="e", width=8)
            label.grid(row=i, column=0, sticky="E")

            value_entry = tk.Entry(Angle_frame, textvariable=var, state='readonly', width=20, justify='center')    
            value_entry.grid(row=i, column=1, sticky="W")

        # Ensure columns are aligned
        Angle_frame.grid_columnconfigure(0, minsize=30)  # Adjust minsize for label column width
        Angle_frame.grid_columnconfigure(1, minsize=100)  # Adjust minsize for value column width


    # ----------------Send/receive data--------------------
    def GUI_Values(self,phi,theta,psi):
        P.km = float(self.km_slider.get())
        P.theta_ref = float(self.theta_ref_slider.get())
        P.theta_kp = float(self.theta_kp_slider.get())
        P.theta_ki = float(self.theta_ki_slider.get())
        P.theta_kd = float(self.theta_kd_slider.get())

        self.phi_val.set(str(phi)[:P.disp_dim])
        self.theta_val.set(str(theta)[:P.disp_dim])
        self.psi_val.set(str(psi)[:P.disp_dim])

    def Flip_Theta(self):
        P.theta_ref = -P.theta_ref
        self.theta_ref_slider.set(P.theta_ref)
