// SPDX-License-Identifier: GPL-2.0-only
/*
 * ucsi-huawei-gaokun - A UCSI driver for HUAWEI Matebook E Go
 *
 * Copyright (C) 2024-2025 Pengyu Luo <mitltlatltl@gmail.com>
 */

#include <drm/bridge/aux-bridge.h>
#include <linux/auxiliary_bus.h>
#include <linux/bitops.h>
#include <linux/completion.h>
#include <linux/container_of.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/of.h>
#include "huawei-gaokun-ec.h"
#include <linux/string.h>
#include <linux/usb/pd_vdo.h>
#include <linux/usb/typec_altmode.h>
#include <linux/usb/typec_dp.h>
#include <linux/usb/typec_mux.h>
#include <linux/workqueue_types.h>

#include "ucsi.h"

#define EC_EVENT_UCSI	0x21
#define EC_EVENT_USB	0x22

#define GAOKUN_UCSI_REGISTER_DELAY	(3 * HZ)
#define GAOKUN_UCSI_RETRY_DELAY		(10 * HZ)
#define GAOKUN_UCSI_MAX_RETRIES		3
#define GAOKUN_HPD_REPLAY_DELAY		(1 * HZ)
#define GAOKUN_HPD_REPLAY_COUNT		3

#define GAOKUN_CCX_MASK		GENMASK(1, 0)
#define GAOKUN_MUX_MASK		GENMASK(3, 2)

#define GAOKUN_DPAM_MASK	GENMASK(3, 0)
#define GAOKUN_HPD_STATE_MASK	BIT(4)
#define GAOKUN_HPD_IRQ_MASK	BIT(5)

#define CCX_TO_ORI(ccx) (++ccx % 3) /* convert ccx to enum typec_orientation */

/* Configuration Channel Extension */
enum gaokun_ucsi_ccx {
	USBC_CCX_NORMAL,
	USBC_CCX_REVERSE,
	USBC_CCX_NONE,
};

enum gaokun_ucsi_mux {
	USBC_MUX_NONE,
	USBC_MUX_USB_2L,
	USBC_MUX_DP_4L,
	USBC_MUX_USB_DP,
};

/* based on pmic_glink_altmode_pin_assignment */
enum gaokun_ucsi_dpam_pan {	/* DP Alt Mode Pin Assignments */
	USBC_DPAM_PAN_NONE,
	USBC_DPAM_PAN_A,	/* Not supported after USB Type-C Standard v1.0b */
	USBC_DPAM_PAN_B,	/* Not supported after USB Type-C Standard v1.0b */
	USBC_DPAM_PAN_C,	/* USBC_DPAM_PAN_C_REVERSE - 6 */
	USBC_DPAM_PAN_D,
	USBC_DPAM_PAN_E,
	USBC_DPAM_PAN_F,	/* Not supported after USB Type-C Standard v1.0b */
	USBC_DPAM_PAN_A_REVERSE,/* Not supported after USB Type-C Standard v1.0b */
	USBC_DPAM_PAN_B_REVERSE,/* Not supported after USB Type-C Standard v1.0b */
	USBC_DPAM_PAN_C_REVERSE,
	USBC_DPAM_PAN_D_REVERSE,
	USBC_DPAM_PAN_E_REVERSE,
	USBC_DPAM_PAN_F_REVERSE,/* Not supported after USB Type-C Standard v1.0b */
};

struct gaokun_ucsi_reg {
	u8 num_ports;
	u8 port_updt;
	u8 port_data[4];
	u8 checksum;
	u8 reserved;
} __packed;

struct gaokun_ucsi_port {
	struct completion usb_ack;
	struct delayed_work no_usb_work;
	struct delayed_work hpd_work;
	spinlock_t lock; /* serializing port resource access */

	struct gaokun_ucsi *ucsi;
	struct auxiliary_device *bridge;
	struct typec_mux *typec_mux;
	struct typec_switch *typec_sw;

	int idx;
	enum gaokun_ucsi_ccx ccx;
	enum gaokun_ucsi_mux mux;
	u8 mode;
	u16 svid;
	u8 hpd_state;
	u8 hpd_irq;
	u8 hpd_replays_left;
};

struct gaokun_ucsi {
	struct gaokun_ec *ec;
	struct ucsi *ucsi;
	struct gaokun_ucsi_port *ports;
	struct device *dev;
	struct delayed_work work;
	struct notifier_block nb;
	u16 version;
	u8 num_ports;
	u8 register_retries;
	bool ports_initialized;
	bool notifier_registered;
	bool ucsi_registered;
};

/* -------------------------------------------------------------------------- */
/* For UCSI */

static int gaokun_ucsi_read_version(struct ucsi *ucsi, u16 *version)
{
	struct gaokun_ucsi *uec = ucsi_get_drvdata(ucsi);

	*version = uec->version;

	return 0;
}

static int gaokun_ucsi_read_cci(struct ucsi *ucsi, u32 *cci)
{
	struct gaokun_ucsi *uec = ucsi_get_drvdata(ucsi);
	u8 buf[GAOKUN_UCSI_READ_SIZE];
	int ret;

	ret = gaokun_ec_ucsi_read(uec->ec, buf);
	if (ret)
		return ret;

	memcpy(cci, buf, sizeof(*cci));

	return 0;
}

static int gaokun_ucsi_read_message_in(struct ucsi *ucsi,
				       void *val, size_t val_len)
{
	struct gaokun_ucsi *uec = ucsi_get_drvdata(ucsi);
	u8 buf[GAOKUN_UCSI_READ_SIZE];
	int ret;

	ret = gaokun_ec_ucsi_read(uec->ec, buf);
	if (ret)
		return ret;

	memcpy(val, buf + GAOKUN_UCSI_CCI_SIZE,
	       min(val_len, GAOKUN_UCSI_MSGI_SIZE));

	return 0;
}

static int gaokun_ucsi_async_control(struct ucsi *ucsi, u64 command)
{
	struct gaokun_ucsi *uec = ucsi_get_drvdata(ucsi);
	u8 buf[GAOKUN_UCSI_WRITE_SIZE] = {};

	memcpy(buf, &command, sizeof(command));

	return gaokun_ec_ucsi_write(uec->ec, buf);
}

static void gaokun_ucsi_update_connector(struct ucsi_connector *con)
{
	struct gaokun_ucsi *uec = ucsi_get_drvdata(con->ucsi);

	if (con->num > uec->num_ports)
		return;

	con->typec_cap.orientation_aware = true;
}

static void gaokun_set_orientation(struct ucsi_connector *con,
				   struct gaokun_ucsi_port *port)
{
	enum gaokun_ucsi_ccx ccx;
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);
	ccx = port->ccx;
	spin_unlock_irqrestore(&port->lock, flags);

	typec_set_orientation(con->port, CCX_TO_ORI(ccx));
}

static void gaokun_ucsi_connector_status(struct ucsi_connector *con)
{
	struct gaokun_ucsi *uec = ucsi_get_drvdata(con->ucsi);
	int idx;

	idx = con->num - 1;
	if (con->num > uec->num_ports) {
		dev_warn(uec->dev, "set orientation out of range: con%d\n", idx);
		return;
	}

	gaokun_set_orientation(con, &uec->ports[idx]);
}

const struct ucsi_operations gaokun_ucsi_ops = {
	.read_version = gaokun_ucsi_read_version,
	.read_cci = gaokun_ucsi_read_cci,
	.poll_cci = gaokun_ucsi_read_cci,
	.read_message_in = gaokun_ucsi_read_message_in,
	.sync_control = ucsi_sync_control_common,
	.async_control = gaokun_ucsi_async_control,
	.update_connector = gaokun_ucsi_update_connector,
	.connector_status = gaokun_ucsi_connector_status,
};

/* -------------------------------------------------------------------------- */
/* For Altmode */

static void gaokun_ucsi_port_update(struct gaokun_ucsi_port *port,
				    const u8 *port_data)
{
	struct gaokun_ucsi *uec = port->ucsi;
	int offset = port->idx * 2; /* every port has 2 Bytes data */
	unsigned long flags;
	u8 dcc, ddi;

	dcc = port_data[offset];
	ddi = port_data[offset + 1];

	spin_lock_irqsave(&port->lock, flags);

	port->ccx = FIELD_GET(GAOKUN_CCX_MASK, dcc);
	port->mux = FIELD_GET(GAOKUN_MUX_MASK, dcc);
	port->mode = FIELD_GET(GAOKUN_DPAM_MASK, ddi);
	port->hpd_state = FIELD_GET(GAOKUN_HPD_STATE_MASK, ddi);
	port->hpd_irq = FIELD_GET(GAOKUN_HPD_IRQ_MASK, ddi);

	/* Mode and SVID are unused; keeping them to make things clearer */
	switch (port->mode) {
	case USBC_DPAM_PAN_C:
	case USBC_DPAM_PAN_C_REVERSE:
		port->mode = DP_PIN_ASSIGN_C; /* correct it for usb later */
		break;
	case USBC_DPAM_PAN_D:
	case USBC_DPAM_PAN_D_REVERSE:
		port->mode = DP_PIN_ASSIGN_D;
		break;
	case USBC_DPAM_PAN_E:
	case USBC_DPAM_PAN_E_REVERSE:
		port->mode = DP_PIN_ASSIGN_E;
		break;
	case USBC_DPAM_PAN_NONE:
		port->mode = TYPEC_STATE_SAFE;
		break;
	default:
		dev_warn(uec->dev, "unknown mode %d\n", port->mode);
		break;
	}

	switch (port->mux) {
	case USBC_MUX_NONE:
		port->svid = 0;
		break;
	case USBC_MUX_USB_2L:
		port->svid = USB_SID_PD;
		port->mode = TYPEC_STATE_USB; /* same as PAN_C, correct it */
		break;
	case USBC_MUX_DP_4L:
	case USBC_MUX_USB_DP:
		port->svid = USB_SID_DISPLAYPORT;
		break;
	default:
		dev_warn(uec->dev, "unknown mux state %d\n", port->mux);
		break;
	}

	spin_unlock_irqrestore(&port->lock, flags);
}

static unsigned int gaokun_ucsi_refresh(struct gaokun_ucsi *uec)
{
	struct gaokun_ucsi_reg ureg;
	unsigned int updates = 0;
	int ret;
	u8 port_updates;
	int idx;

	ret = gaokun_ec_ucsi_get_reg(uec->ec, &ureg);
	if (ret)
		return 0;

	uec->num_ports = ureg.num_ports;
	port_updates = ureg.port_updt;
	for (idx = 0; idx < ureg.num_ports; idx++) {
		if (!(port_updates & BIT(idx)))
			continue;
		gaokun_ucsi_port_update(&uec->ports[idx], ureg.port_data);
		updates |= BIT(idx);
	}

	return updates;
}

static unsigned long gaokun_ucsi_typec_mux_mode(const struct gaokun_ucsi_port *port)
{
	switch (port->svid) {
	case USB_SID_DISPLAYPORT:
		switch (port->mode) {
		case DP_PIN_ASSIGN_C:
			return TYPEC_DP_STATE_C;
		case DP_PIN_ASSIGN_D:
			return TYPEC_DP_STATE_D;
		case DP_PIN_ASSIGN_E:
			return TYPEC_DP_STATE_E;
		default:
			return TYPEC_STATE_SAFE;
		}
	case USB_SID_PD:
		return TYPEC_STATE_USB;
	default:
		return TYPEC_STATE_SAFE;
	}
}

static void gaokun_ucsi_handle_altmode(struct gaokun_ucsi_port *port)
{
	struct gaokun_ucsi *uec = port->ucsi;
	struct typec_mux_state state = {};
	struct typec_altmode dp_alt = {};
	struct typec_displayport_data dp_data = {};
	int idx = port->idx;
	int ret;

	if (idx >= uec->ucsi->cap.num_connectors) {
		dev_warn(uec->dev, "altmode port out of range: %d\n", idx);
		return;
	}

	if (port->typec_sw) {
		ret = typec_switch_set(port->typec_sw, CCX_TO_ORI(port->ccx));
		if (ret)
			dev_warn(uec->dev, "failed to set orientation switch for port %d: %d\n",
				 idx, ret);
	}

	if (port->typec_mux) {
		state.mode = gaokun_ucsi_typec_mux_mode(port);

		if (port->svid == USB_SID_DISPLAYPORT) {
			dp_alt.svid = USB_TYPEC_DP_SID;
			dp_alt.mode = USB_TYPEC_DP_MODE;
			state.alt = &dp_alt;

			dp_data.status = DP_STATUS_ENABLED;
			if (port->hpd_state)
				dp_data.status |= DP_STATUS_HPD_STATE;
			if (port->hpd_irq)
				dp_data.status |= DP_STATUS_IRQ_HPD;
			dp_data.conf = DP_CONF_UFP_U_AS_UFP_D |
				       DP_CONF_SET_PIN_ASSIGN(port->mode);
			state.data = &dp_data;
		}

		ret = typec_mux_set(port->typec_mux, &state);
		if (ret)
			dev_warn(uec->dev, "failed to set typec mux for port %d: mode=0x%lx ret=%d\n",
				 idx, state.mode, ret);
	}

	/* UCSI callback .connector_status() have set orientation */
	if (port->bridge)
		drm_aux_hpd_bridge_notify(&port->bridge->dev,
					  port->hpd_state ?
					  connector_status_connected :
					  connector_status_disconnected);

	if (port->hpd_state && port->svid == USB_SID_DISPLAYPORT) {
		port->hpd_replays_left = GAOKUN_HPD_REPLAY_COUNT;
		mod_delayed_work(system_wq, &port->hpd_work, GAOKUN_HPD_REPLAY_DELAY);
	} else {
		port->hpd_replays_left = 0;
		cancel_delayed_work(&port->hpd_work);
	}

	gaokun_ec_ucsi_pan_ack(uec->ec, port->idx);
}

static void gaokun_ucsi_hpd_replay_worker(struct work_struct *work)
{
	struct gaokun_ucsi_port *port;

	port = container_of(work, struct gaokun_ucsi_port, hpd_work.work);

	if (!port->bridge || !port->hpd_state || port->svid != USB_SID_DISPLAYPORT)
		return;

	drm_aux_hpd_bridge_notify(&port->bridge->dev, connector_status_connected);

	if (port->hpd_replays_left > 1) {
		port->hpd_replays_left--;
		mod_delayed_work(system_wq, &port->hpd_work, GAOKUN_HPD_REPLAY_DELAY);
	} else {
		port->hpd_replays_left = 0;
	}
}

static unsigned int gaokun_ucsi_altmode_notify_ind(struct gaokun_ucsi *uec)
{
	unsigned int updates;
	int idx;

	if (!uec->ucsi_registered || !uec->ucsi->connector) {
		dev_warn_ratelimited(uec->dev,
				     "ucsi connector is not initialized yet, acking pending event\n");
		gaokun_ec_ucsi_pan_ack(uec->ec, GAOKUN_UCSI_NO_PORT_UPDATE);
		return 0;
	}

	updates = gaokun_ucsi_refresh(uec);
	if (!updates) {
		gaokun_ec_ucsi_pan_ack(uec->ec, GAOKUN_UCSI_NO_PORT_UPDATE);
		return 0;
	}

	for (idx = 0; idx < uec->num_ports; idx++) {
		if (!(updates & BIT(idx)))
			continue;
		gaokun_ucsi_handle_altmode(&uec->ports[idx]);
	}

	return updates;
}

static void gaokun_ucsi_no_usb_event_worker(struct work_struct *work)
{
	struct gaokun_ucsi_port *port;
	struct gaokun_ucsi *uec;

	port = container_of(work, struct gaokun_ucsi_port, no_usb_work.work);
	uec = port->ucsi;

	if (completion_done(&port->usb_ack))
		return;

	dev_warn(uec->dev, "No USB EVENT, triggered by UCSI EVENT");
	gaokun_ucsi_altmode_notify_ind(uec);
}

static void gaokun_ucsi_complete_usb_ack(struct gaokun_ucsi *uec, unsigned int updates)
{
	struct gaokun_ucsi_port *port;
	int idx;

	for (idx = 0; idx < uec->num_ports; idx++) {
		if (!(updates & BIT(idx)))
			continue;
		port = &uec->ports[idx];
		if (!completion_done(&port->usb_ack))
			complete_all(&port->usb_ack);
	}
}

/*
 * Apply the EC-reported switch and mux state only after the USB follow-up
 * event arrives. If that event never comes, force a refresh after a timeout.
 */
static void gaokun_ucsi_schedule_no_usb_check(struct gaokun_ucsi *uec, int idx)
{
	struct gaokun_ucsi_port *port;

	port = &uec->ports[idx];
	reinit_completion(&port->usb_ack);
	mod_delayed_work(system_wq, &port->no_usb_work, 2 * HZ);
}

static int gaokun_ucsi_notify(struct notifier_block *nb,
			      unsigned long action, void *data)
{
	u32 cci;
	struct gaokun_ucsi *uec = container_of(nb, struct gaokun_ucsi, nb);
	unsigned int updates;

	switch (action) {
	case EC_EVENT_USB:
		/*
		 * USB follow-up events are port-specific. Completing all ports
		 * here lets activity on one connector mask a missing follow-up
		 * event on the other connector.
		 */
		updates = gaokun_ucsi_altmode_notify_ind(uec);
		gaokun_ucsi_complete_usb_ack(uec, updates);
		return NOTIFY_OK;

	case EC_EVENT_UCSI:
		if (gaokun_ucsi_read_cci(uec->ucsi, &cci)) {
			dev_warn(uec->dev, "failed to read CCI for UCSI event\n");
			return NOTIFY_OK;
		}

		if (UCSI_CCI_CONNECTOR(cci))
			gaokun_ucsi_schedule_no_usb_check(uec,
							  UCSI_CCI_CONNECTOR(cci) - 1);

		ucsi_notify_common(uec->ucsi, cci);

		return NOTIFY_OK;

	default:
		return NOTIFY_DONE;
	}
}

static int gaokun_ucsi_ports_init(struct gaokun_ucsi *uec)
{
	struct gaokun_ucsi_port *ucsi_port;
	struct device *dev = uec->dev;
	struct fwnode_handle *fwnode;
	int i, ret, num_ports;
	u32 port;

	if (uec->ports_initialized)
		return 0;

	num_ports = 0;
	device_for_each_child_node(dev, fwnode)
		num_ports++;

	if (num_ports <= 0) {
		dev_err(dev, "no connector child nodes found for UCSI bridge setup\n");
		return -ENODEV;
	}

	uec->num_ports = num_ports;
	uec->ports = devm_kcalloc(dev, num_ports, sizeof(*(uec->ports)),
				  GFP_KERNEL);
	if (!uec->ports)
		return -ENOMEM;

	for (i = 0; i < num_ports; ++i) {
		ucsi_port = &uec->ports[i];
		ucsi_port->ccx = USBC_CCX_NONE;
		ucsi_port->idx = i;
		ucsi_port->ucsi = uec;
		init_completion(&ucsi_port->usb_ack);
		INIT_DELAYED_WORK(&ucsi_port->no_usb_work,
				  gaokun_ucsi_no_usb_event_worker);
		INIT_DELAYED_WORK(&ucsi_port->hpd_work,
				  gaokun_ucsi_hpd_replay_worker);
		spin_lock_init(&ucsi_port->lock);
	}

	device_for_each_child_node(dev, fwnode) {
		ret = fwnode_property_read_u32(fwnode, "reg", &port);
		if (ret < 0) {
			dev_err(dev, "missing reg property of %pOFn\n", fwnode);
			fwnode_handle_put(fwnode);
			return ret;
		}

		if (port >= num_ports) {
			dev_warn(dev, "invalid connector number %d, ignoring\n", port);
			continue;
		}

		ucsi_port = &uec->ports[port];
		ucsi_port->bridge = devm_drm_dp_hpd_bridge_alloc(dev, to_of_node(fwnode));
		if (IS_ERR(ucsi_port->bridge)) {
			dev_err_probe(dev, PTR_ERR(ucsi_port->bridge),
				      "failed to allocate DP HPD bridge for connector %u\n",
				      port);
			fwnode_handle_put(fwnode);
			return PTR_ERR(ucsi_port->bridge);
		}

		ucsi_port->typec_mux = fwnode_typec_mux_get(fwnode);
		if (IS_ERR(ucsi_port->typec_mux)) {
			ret = PTR_ERR(ucsi_port->typec_mux);
			dev_err_probe(dev, ret,
				      "failed to get typec mux for connector %u\n",
				      port);
			fwnode_handle_put(fwnode);
			return ret;
		}

		ucsi_port->typec_sw = fwnode_typec_switch_get(fwnode);
		if (IS_ERR(ucsi_port->typec_sw)) {
			ret = PTR_ERR(ucsi_port->typec_sw);
			dev_err_probe(dev, ret,
				      "failed to get typec switch for connector %u\n",
				      port);
			fwnode_handle_put(fwnode);
			return ret;
		}
	}

	for (i = 0; i < num_ports; i++) {
		if (!uec->ports[i].bridge)
			continue;

		ret = devm_drm_dp_hpd_bridge_add(dev, uec->ports[i].bridge);
		if (ret) {
			dev_err_probe(dev, ret,
				      "failed to add DP HPD bridge for connector %d\n", i);
			return ret;
		}
	}

	uec->ports_initialized = true;

	return 0;
}

static int gaokun_ucsi_ec_init(struct gaokun_ucsi *uec)
{
	struct gaokun_ucsi_reg ureg = {};
	int ret;

	ret = gaokun_ec_ucsi_get_reg(uec->ec, &ureg);
	if (ret)
		return ret;

	if (ureg.num_ports <= 0)
		return -ENODEV;

	if (ureg.num_ports > uec->num_ports)
		return -EINVAL;

	uec->num_ports = ureg.num_ports;

	return 0;
}

static void gaokun_ucsi_register_worker(struct work_struct *work)
{
	struct gaokun_ucsi *uec;
	struct ucsi *ucsi;
	int ret;

	uec = container_of(work, struct gaokun_ucsi, work.work);
	ucsi = uec->ucsi;

	ret = gaokun_ucsi_ports_init(uec);
	if (ret)
		goto retry;

	ret = gaokun_ucsi_ec_init(uec);
	if (ret)
		goto retry;

	ret = ucsi_register(ucsi);
	if (ret) {
		dev_err_probe(ucsi->dev, ret, "ucsi register failed\n");
		goto retry;
	}

	uec->ucsi_registered = true;

	ret = gaokun_ec_register_notify(uec->ec, &uec->nb);
	if (ret) {
		dev_err_probe(ucsi->dev, ret, "notifier register failed\n");
		ucsi_unregister(ucsi);
		uec->ucsi_registered = false;
		goto retry;
	}

	uec->notifier_registered = true;

	return;

retry:
	if (++uec->register_retries > GAOKUN_UCSI_MAX_RETRIES) {
		dev_err(uec->dev, "giving up on UCSI registration after %u attempts\n",
			uec->register_retries);
		return;
	}

	dev_warn(uec->dev, "retrying UCSI registration in %u seconds (attempt %u/%u)\n",
		 jiffies_to_msecs(GAOKUN_UCSI_RETRY_DELAY) / 1000,
		 uec->register_retries, GAOKUN_UCSI_MAX_RETRIES);
	schedule_delayed_work(&uec->work, GAOKUN_UCSI_RETRY_DELAY);
}

static int gaokun_ucsi_probe(struct auxiliary_device *adev,
			     const struct auxiliary_device_id *id)
{
	struct gaokun_ec *ec = adev->dev.platform_data;
	struct device *dev = &adev->dev;
	struct gaokun_ucsi *uec;
	int ret;

	uec = devm_kzalloc(dev, sizeof(*uec), GFP_KERNEL);
	if (!uec)
		return -ENOMEM;

	uec->ec = ec;
	uec->dev = dev;
	uec->version = UCSI_VERSION_1_0;
	uec->nb.notifier_call = gaokun_ucsi_notify;

	INIT_DELAYED_WORK(&uec->work, gaokun_ucsi_register_worker);

	ret = gaokun_ucsi_ports_init(uec);
	if (ret)
		return dev_err_probe(dev, ret, "failed to initialize UCSI ports\n");

	uec->ucsi = ucsi_create(dev, &gaokun_ucsi_ops);
	if (IS_ERR(uec->ucsi))
		return dev_err_probe(dev, PTR_ERR(uec->ucsi),
				     "failed to create UCSI instance\n");

	ucsi_set_drvdata(uec->ucsi, uec);
	auxiliary_set_drvdata(adev, uec);

	/*
	 * Do not touch EC UCSI state or register for EC notifications until the
	 * platform has settled; boot-time EC events can otherwise race with
	 * UCSI initialization and leave the EC interrupt storming.
	 */
	schedule_delayed_work(&uec->work, GAOKUN_UCSI_REGISTER_DELAY);

	return 0;
}

static void gaokun_ucsi_remove(struct auxiliary_device *adev)
{
	struct gaokun_ucsi *uec = auxiliary_get_drvdata(adev);
	int i;

	cancel_delayed_work_sync(&uec->work);

	if (uec->ports_initialized) {
		for (i = 0; i < uec->num_ports; i++) {
			cancel_delayed_work_sync(&uec->ports[i].no_usb_work);
			cancel_delayed_work_sync(&uec->ports[i].hpd_work);
			typec_switch_put(uec->ports[i].typec_sw);
			typec_mux_put(uec->ports[i].typec_mux);
		}
	}

	if (uec->notifier_registered)
		gaokun_ec_unregister_notify(uec->ec, &uec->nb);

	if (uec->ucsi_registered)
		ucsi_unregister(uec->ucsi);

	ucsi_destroy(uec->ucsi);
}

static const struct auxiliary_device_id gaokun_ucsi_id_table[] = {
	{ .name = GAOKUN_MOD_NAME "." GAOKUN_DEV_UCSI, },
	{}
};
MODULE_DEVICE_TABLE(auxiliary, gaokun_ucsi_id_table);

static struct auxiliary_driver gaokun_ucsi_driver = {
	.name = GAOKUN_DEV_UCSI,
	.id_table = gaokun_ucsi_id_table,
	.probe = gaokun_ucsi_probe,
	.remove = gaokun_ucsi_remove,
};

module_auxiliary_driver(gaokun_ucsi_driver);

MODULE_DESCRIPTION("HUAWEI Matebook E Go UCSI driver");
MODULE_LICENSE("GPL");
