m(t)  # simulate sensor noise, will use in future assignments
        u, xhat = controller.update(r, y + n)  # u