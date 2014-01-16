

struct fm_ld_drv_register {
    /**
    * @priv_data: priv data holder for the FMR driver. This is filled by
    * the FMR driver data during registration, and sent back on
    * ld_drv_reg_complete_cb and cmd_handler members.
    */
    void *priv_data;

    /**
    * @ ld_drv_reg_complete_cb: Line Discipline Driver registration complete
    * callback.
    *
    * This function is called by the Line discipline driver when
    * registration is completed.
    *
    * Arg parameter is the fm driver private data
    * data parameter is status of LD registration In
    * progress/Registered/Unregistered.
    */
    void (*ld_drv_reg_complete_cb) (void *arg, char data);

    /**
    * @fm_cmd_handler: BT Cmd/Event and IRQ informer
    *
    * This function is called by the line descipline driver, to signal the FM
    * Radio driver on host when FM packet is available. This includes command
    * complete and IRQ forwarding. However the processing of the buff contents,
    * do not happen in the LD driver context.
    *
    * arg parameter is the fm driver private data
    * data parameter is status of LD registration In
    * Progress/Registered/Unregistered.
    */
    long (*fm_cmd_handler) (void *arg, struct sk_buff *);

    /**
    * @fm_cmd_write: FM Command Send function
    *
    * This field should be populated by the LD driver. This function is used by
    * FM Radio driver to send the command buffer
    */
    long (*fm_cmd_write) (struct sk_buff *skb);
};


extern long register_fmdrv_to_ld_driv(struct fm_ld_drv_register *fm_ld_drv_reg);

extern long unregister_fmdrv_from_ld_driv(struct fm_ld_drv_register *fm_ld_drv_reg);
