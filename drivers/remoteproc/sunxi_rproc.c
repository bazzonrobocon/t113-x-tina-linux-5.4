/*
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Copyright © 2020-2025, Allwinnertech
 *
 * This file is provided under a dual BSD/GPL license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/* #define DEBUG */
#include <linux/arm-smccc.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/remoteproc.h>
#include <linux/io.h>
#include <linux/mailbox_client.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_reserved_mem.h>
#include <linux/pm_wakeirq.h>
#include <linux/regmap.h>
#include <linux/remoteproc.h>
#include <linux/slab.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/clk.h>
#include "remoteproc_internal.h"
#include "sunxi_rproc_boot.h"
#include "remoteproc_elf_helpers.h"
#include "remoteproc_elf_helpers.h"
#include "sunxi_rproc_firmware.h"

#define SUNXI_RPROC_VERSION "2.2.1"

#define MBOX_NB_VQ		2

static LIST_HEAD(sunxi_rproc_list);
struct sunxi_mbox {
	struct mbox_chan *chan;
	struct mbox_client client;
	struct work_struct vq_work;
	int vq_id;
};

struct sunxi_rproc {
	struct sunxi_rproc_priv *rproc_priv;  /* dsp/riscv private resources */
	struct sunxi_mbox mb;
	struct workqueue_struct *workqueue;
	struct list_head list;
	struct rproc *rproc;
	bool is_booted;
	char *name;
};

int simulator_debug;
module_param(simulator_debug, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(simulator_debug, "Debug for simulator");

static int sunxi_rproc_pa_to_da(struct rproc *rproc, phys_addr_t pa, u64 *da)
{
	struct device *dev = rproc->dev.parent;
	struct sunxi_rproc *chip = rproc->priv;
	struct sunxi_rproc_memory_mapping *map;
	int i;

	/*
	 * Maybe there are multiple DAs corresponding to one PA.
	 * Here we only return the first matching one in the map table.
	 */
	for (i = 0; i < chip->rproc_priv->mem_maps_cnt; i++) {
		map = &chip->rproc_priv->mem_maps[i];
		if (pa < map->pa || pa >= map->pa + map->len)
			continue;
		*da = pa - map->pa + map->da;
		dev_dbg(dev, "translate pa %pa to da 0x%llx\n", &pa, *da);
		return 0;
	}

	dev_err(dev, "Failed to translate pa %pa to da\n", &pa);
	return -EINVAL;
}

static int sunxi_rproc_da_to_pa(struct rproc *rproc, u64 da, phys_addr_t *pa)
{
	struct device *dev = rproc->dev.parent;
	struct sunxi_rproc *chip = rproc->priv;
	struct sunxi_rproc_memory_mapping *map;
	int i;

	for (i = 0; i < chip->rproc_priv->mem_maps_cnt; i++) {
		map = &chip->rproc_priv->mem_maps[i];
		if (da < map->da || da >= map->da + map->len)
			continue;
		*pa = da - map->da + map->pa;
		dev_dbg(dev, "translate da 0x%llx to pa %pa\n", da, pa);
		return 0;
	}

	dev_err(dev, "Failed to translate da 0x%llx to pa\n", da);
	return -EINVAL;
}

static int sunxi_rproc_mem_alloc(struct rproc *rproc,
				 struct rproc_mem_entry *mem)
{
	struct device *dev = rproc->dev.parent;
	void *va;

	dev_dbg(dev, "map memory: %pad+%x\n", &mem->dma, mem->len);
	va = ioremap_wc(mem->dma, mem->len);
	if (IS_ERR_OR_NULL(va)) {
		dev_err(dev, "Unable to map memory region: %pad+%x\n",
			&mem->dma, mem->len);
		return -ENOMEM;
	}

	/* Update memory entry va */
	mem->va = va;

	return 0;
}

static int sunxi_rproc_mem_release(struct rproc *rproc,
				   struct rproc_mem_entry *mem)
{
	dev_dbg(rproc->dev.parent, "unmap memory: %pad\n", &mem->dma);
	iounmap(mem->va);

	return 0;
}

static void *sunxi_rproc_da_to_va(struct rproc *rproc, u64 da, int len)
{
	struct device *dev = rproc->dev.parent;
	struct rproc_mem_entry *carveout;
	void *ptr = NULL;
	phys_addr_t pa;
	int ret;

	dev_dbg(dev, "%s,%d\n", __func__, __LINE__);

	/* first step: translate da to pa */
	ret = sunxi_rproc_da_to_pa(rproc, da, &pa);
	if (ret) {
		dev_err(dev, "invalid da 0x%llx\n", da);
		return NULL;
	}

	/* second step: get va from carveouts via pa */
	list_for_each_entry(carveout, &rproc->carveouts, node) {
		if ((pa >= carveout->dma) && (pa < carveout->dma + carveout->len)) {
			ptr = carveout->va + (pa - carveout->dma);
			return ptr;
		}
	}
	return NULL;
}

static void sunxi_rproc_mb_vq_work(struct work_struct *work)
{
	struct sunxi_mbox *mb = container_of(work, struct sunxi_mbox, vq_work);
	struct rproc *rproc = dev_get_drvdata(mb->client.dev);

	dev_dbg(&rproc->dev, "%s,%d\n", __func__, __LINE__);

	/*
	 * We put the data receiving and processing part
	 * of the virtqueue in the bottom half.
	 */
	if (rproc_vq_interrupt(rproc, mb->vq_id) == IRQ_NONE)
		dev_dbg(&rproc->dev, "no message found in vq%d\n", mb->vq_id);
}

static void sunxi_rproc_mb_rx_callback(struct mbox_client *cl, void *data)
{
	struct rproc *rproc = dev_get_drvdata(cl->dev);
	struct sunxi_mbox *mb = container_of(cl, struct sunxi_mbox, client);
	struct sunxi_rproc *chip = rproc->priv;

	dev_dbg(&rproc->dev, "%s,%d name = arm-kick, vq_id = 0x%x\n", __func__, __LINE__, mb->vq_id);

	/*
	 * Data is sent from remote processor,
	 * which represents the virtqueue ID.
	 */
	mb->vq_id = *(u32 *)data;

	queue_work(chip->workqueue, &mb->vq_work);
}

static void sunxi_rproc_mb_tx_done(struct mbox_client *cl, void *msg, int r)
{
	struct rproc *rproc = dev_get_drvdata(cl->dev);
	struct sunxi_mbox *mb = container_of(cl, struct sunxi_mbox, client);

	dev_dbg(&rproc->dev, "%s,%d name = arm-kick, vq_id = 0x%x\n", __func__, __LINE__, mb->vq_id);
	devm_kfree(&rproc->dev, msg);
}

static int sunxi_rproc_request_mbox(struct rproc *rproc)
{
	struct sunxi_rproc *chip = rproc->priv;
	struct device *dev = &rproc->dev;

	/* Initialise mailbox structure table */
	chip->mb.client.rx_callback = sunxi_rproc_mb_rx_callback;
	chip->mb.client.tx_done = sunxi_rproc_mb_tx_done;
	chip->mb.client.tx_block = false;
	chip->mb.client.dev = dev->parent;

	chip->mb.chan = mbox_request_channel_byname(&chip->mb.client, "arm-kick");
	if (IS_ERR(chip->mb.chan)) {
		if (PTR_ERR(chip->mb.chan) == -EPROBE_DEFER)
			goto err_probe;
		dev_warn(dev, "cannot get arm-kick mbox\n");
		chip->mb.chan = NULL;
	}

	INIT_WORK(&chip->mb.vq_work, sunxi_rproc_mb_vq_work);

	return 0;

err_probe:
	mbox_free_channel(chip->mb.chan);
	return -EPROBE_DEFER;
}

static void sunxi_rproc_free_mbox(struct rproc *rproc)
{
	struct sunxi_rproc *chip = rproc->priv;

	mbox_free_channel(chip->mb.chan);
	chip->mb.chan = NULL;
}

static int sunxi_rproc_attach(struct rproc *rproc)
{
	struct sunxi_rproc *chip = rproc->priv;
	struct sunxi_rproc_priv *rproc_priv = chip->rproc_priv;
	int ret;

	ret = sunxi_rproc_priv_attach(rproc_priv);
	if (ret) {
		dev_err(rproc_priv->dev, "start remoteproc error\n");
		return ret;
	}

	return 0;
}

static int sunxi_rproc_start(struct rproc *rproc)
{
	struct sunxi_rproc *chip = rproc->priv;
	struct sunxi_rproc_priv *rproc_priv = chip->rproc_priv;
	int ret;

	ret = sunxi_rproc_priv_start(rproc_priv);
	if (ret) {
		dev_err(rproc_priv->dev, "start remoteproc error\n");
		return ret;
	}

	return 0;
}

static int sunxi_rproc_stop(struct rproc *rproc)
{
	struct sunxi_rproc *chip = rproc->priv;
	struct sunxi_rproc_priv *rproc_priv = chip->rproc_priv;
	int ret;

	ret = sunxi_rproc_priv_stop(rproc_priv);
	if (ret) {
		dev_err(rproc_priv->dev, "stop remoteproc error\n");
		return ret;
	}

	return 0;
}

static int sunxi_rproc_parse_fw(struct rproc *rproc, const struct firmware *fw)
{

	int ret = 0;
	struct sunxi_rproc *chip = rproc->priv;
	const u8 *elf_data = fw->data;
	const void *ehdr;
	u8 class = fw_elf_get_class(fw);

	struct device *dev = rproc->dev.parent;
	struct device_node *np = dev->of_node;
	struct of_phandle_iterator it;
	struct rproc_mem_entry *mem;
	struct reserved_mem *rmem;
	int index = 0;
	u64 da;

	ehdr = elf_data;
	dev_dbg(dev, "%s,%d\n", __func__, __LINE__);

	ret = of_phandle_iterator_init(&it, np, "memory-region", NULL, 0);
	if (ret) {
		dev_err(dev, "memory-region iterator init fail %d\n", ret);
		return -ENODEV;
	}

	while (of_phandle_iterator_next(&it) == 0) {
		rmem = of_reserved_mem_lookup(it.node);
		if (!rmem) {
			dev_err(dev, "unable to acquire memory-region\n");
			return -EINVAL;
		}

		ret = sunxi_rproc_pa_to_da(rproc, rmem->base, &da);
		if (ret) {
			dev_err(dev, "memory region not valid: %pa\n", &rmem->base);
			return -EINVAL;
		}

		/* No need to map vdev buffer */
		if (0 == strcmp(it.node->name, "vdev0buffer")) {
			mem = rproc_of_resm_mem_entry_init(dev, index,
							   rmem->size,
							   da,
							   it.node->name);
			/*
			 * The rproc_of_resm_mem_entry_init didn't save the
			 * physical address. Here we save it manually.
			 */
			if (mem)
				mem->dma = (dma_addr_t)rmem->base;
		} else {
			mem = rproc_mem_entry_init(dev, NULL,
						   (dma_addr_t)rmem->base,
						   rmem->size, da,
						   sunxi_rproc_mem_alloc,
						   sunxi_rproc_mem_release,
						   it.node->name);
			if (mem)
				rproc_coredump_add_segment(rproc, da,
							   rmem->size);
		}

		if (!mem)
			return -ENOMEM;

		rproc_add_carveout(rproc, mem);
		index++;
	}

	chip->rproc_priv->pc_entry = elf_hdr_get_e_entry(class, ehdr);
	/* check segment name, such as .resource_table */
	ret = rproc_elf_load_rsc_table(rproc, fw);
	if (ret != 0) {
		rproc->cached_table = NULL;
		rproc->table_ptr = NULL;
		rproc->table_sz = 0;
		dev_warn(&rproc->dev, "no resource table found for this firmware\n");
		/* set ret 0 to avoid program not run without resource_table */
		ret = 0;
	}
	return ret;
}

static void sunxi_rproc_kick(struct rproc *rproc, int vqid)
{
	struct sunxi_rproc *chip = rproc->priv;
	u32 *msg = NULL;
	int err;

	dev_dbg(&rproc->dev, "%s,%d vqid = 0x%x\n", __func__, __LINE__, vqid);

	if (WARN_ON(vqid >= MBOX_NB_VQ))
		return;

	/*
	 * Because of the implementation of sunxi msgbox(mailbox controller),
	 * the type of mailbox message should be u32.
	 */
	msg = devm_kzalloc(&rproc->dev, sizeof(*msg), GFP_KERNEL);
	if (!msg)
		return;

	*msg = vqid;

	/* Remeber to free msg in mailbox tx_done callback */
	err = mbox_send_message(chip->mb.chan, (void *)msg);
	if (err < 0)
		dev_err(&rproc->dev, "%s,%d kick err:%d\n",
			__func__, __LINE__, err);
	return;
}

static int sunxi_rproc_elf_find_section(struct rproc *rproc,
					 const struct firmware *fw,
					 const char *section_name,
					 const void **find_shdr)
{
	struct device *dev = &rproc->dev;
	const void *shdr, *name_table_shdr;
	int i;
	const char *name_table;
	const u8 *elf_data = (void *)fw->data;
	u8 class = fw_elf_get_class(fw);
	const void *ehdr = elf_data;
	u16 shnum = elf_hdr_get_e_shnum(class, ehdr);
	u32 elf_shdr_get_size = elf_size_of_shdr(class);
	u16 shstrndx = elf_hdr_get_e_shstrndx(class, ehdr);

	shdr = elf_data + elf_hdr_get_e_shoff(class, ehdr);
	name_table_shdr = shdr + (shstrndx * elf_shdr_get_size);
	name_table = elf_data + elf_shdr_get_sh_offset(class, name_table_shdr);

	for (i = 0; i < shnum; i++, shdr += elf_shdr_get_size) {
		u64 size = elf_shdr_get_sh_size(class, shdr);
		u32 name = elf_shdr_get_sh_name(class, shdr);

		if (strcmp(name_table + name, section_name))
			continue;

		*find_shdr = shdr;
		dev_dbg(dev, "%s,%d %s addr 0x%llx, size 0x%llx\n",
			__func__, __LINE__, section_name, elf_shdr_get_sh_addr(class, shdr), size);

		return 0;
	}

	return -EINVAL;
}

static int sunxi_rproc_elf_load_segments(struct rproc *rproc, const struct firmware *fw)
{

	struct device *dev = &rproc->dev;
	struct sunxi_rproc *chip = rproc->priv;
	u8 class = fw_elf_get_class(fw);
	const void *ehdr;
	const void *phdr;
	const void *shdr;
	int i, ret = 0;
	const u8 *elf_data = fw->data;
	u64 offset, da, memsz, filesz;
	u32 type;
	void *ptr;
	u16 phnum;
	u32 elf_phdr_get_size = elf_size_of_phdr(class);

	ehdr = elf_data;
	phdr = elf_data + elf_hdr_get_e_phoff(class, ehdr);
	phnum = elf_hdr_get_e_phnum(class, ehdr);
	ret = sunxi_rproc_elf_find_section(rproc, fw, ".version_table", &shdr);
	if (ret) {
		dev_warn(dev, "%s,%d no  find segments version_table\n", __func__, __LINE__);
		/* Lack of ".version_table" should not be assumed as an error */
		ret = 0;
	} else {
		dev_info(dev, "the version: %s\n", elf_data + elf_shdr_get_sh_offset(class, shdr));
	}

	if (simulator_debug) {
		dev_dbg(dev, "%s,%d only load .resource_table data\n",
				__func__, __LINE__);

		ret = sunxi_rproc_elf_find_section(rproc, fw, ".resource_table", &shdr);
		if (ret) {
			dev_err(dev, "%s,%d find segments err\n", __func__, __LINE__);
			return ret;
		}

		da = elf_shdr_get_sh_addr(class, shdr);
		memsz = elf_shdr_get_sh_size(class, shdr);
		ptr = rproc_da_to_va(rproc, da, memsz);
		if (!ptr) {
			dev_err(dev, "bad phdr da 0x%llx mem 0x%llx\n", da, memsz);
			return -EINVAL;
		}
		memcpy(ptr, elf_data + elf_shdr_get_sh_offset(class, shdr), memsz);

		return 0;
	}

	sunxi_rproc_priv_set_localram(chip->rproc_priv, 1);
	dev_dbg(dev, "%s,%d\n", __func__, __LINE__);

	for (i = 0; i < phnum; i++, phdr += elf_phdr_get_size) {
		da = elf_phdr_get_p_paddr(class, phdr);
		memsz = elf_phdr_get_p_memsz(class, phdr);
		filesz = elf_phdr_get_p_filesz(class, phdr);
		offset = elf_phdr_get_p_offset(class, phdr);
		type = elf_phdr_get_p_type(class, phdr);

		if (type != PT_LOAD)
			continue;

		dev_dbg(dev, "phdr: type %d da 0x%llx memsz 0x%llx filesz 0x%llx\n",
			type, da, memsz, filesz);

		if ((memsz == 0) || (filesz == 0))
			continue;

		if (filesz > memsz) {
			dev_err(dev, "bad phdr filesz 0x%llx memsz 0x%llx\n",
				filesz, memsz);
			ret = -EINVAL;
			break;
		}

		if (offset + filesz > fw->size) {
			dev_err(dev, "truncated fw: need 0x%llx avail 0x%zx\n",
				offset + filesz, fw->size);
			ret = -EINVAL;
			break;
		}

		/* grab the kernel address for this device address */
		ptr = rproc_da_to_va(rproc, da, memsz);
		if (!ptr) {
			dev_err(dev, "bad phdr da 0x%llx mem 0x%llx\n", da,
				memsz);
			ret = -EINVAL;
			break;
		}

		/* put the segment where the remote processor expects it */
		if (filesz)
			memcpy(ptr, elf_data + offset, filesz);

		if (memsz > filesz)
			memset(ptr + filesz, 0, memsz - filesz);
	}

	return ret;
}

static struct resource_table *sunxi_rproc_rsc_table(struct rproc *rproc,
					     const struct firmware *fw)
{
	dev_dbg(&rproc->dev, "%s,%d\n", __func__, __LINE__);
	return rproc_elf_find_loaded_rsc_table(rproc, fw);
}

int sunxi_rproc_request_firmware(struct rproc *rproc, const struct firmware **fw)
{
	struct device *dev = rproc->dev.parent;
	int ret;

	dev_dbg(&rproc->dev, "%s,%d\n", __func__, __LINE__);
	ret = sunxi_request_firmware(fw, rproc->firmware, dev);
	return ret;
}

static void suxni_rproc_auto_boot_callback(const struct firmware *fw, void *context)
{
	struct rproc *rproc = context;

	if (rproc->auto_boot)
		rproc_boot(rproc);

	release_firmware(fw);
}

static int suxni_rproc_trigger_auto_boot(struct rproc *rproc)
{
	int ret = 0;

	ret = sunxi_request_firmware_nowait(rproc->firmware, &rproc->dev,
					GFP_KERNEL, rproc, suxni_rproc_auto_boot_callback);
	if (ret < 0)
		dev_err(&rproc->dev, "sunxi_request_firmware_nowait err: %d\n", ret);

	return 0;
}

static u64 suxni_rproc_elf_get_boot_addr(struct rproc *rproc, const struct firmware *fw)
{
	u64 data = 0;
	data = rproc_elf_get_boot_addr(rproc, fw);
	dev_dbg(&rproc->dev, "%s,%d elf boot addr = 0x%llx\n", __func__, __LINE__, data);
	return data;
}

#ifdef CONFIG_SUNXI_REMOTEPROC_TRACE_DEV
extern struct dentry *sunxi_rproc_create_aw_trace_file(const char *name, struct rproc *rproc,
				       struct rproc_debug_trace *trace);

static int sunxi_rproc_handle_aw_trace(struct rproc *rproc, void *ptr,
			       int offset, int avail)
{
	struct fw_rsc_aw_trace *rsc = ptr;
	struct rproc_debug_trace *trace;
	struct device *dev = rproc->dev.parent;
	char name[64];

	if (sizeof(*rsc) > avail) {
		dev_err(dev, "trace rsc is truncated\n");
		return -EINVAL;
	}

	/* make sure reserved bytes are zeroes */
	if (rsc->reserved) {
		dev_err(dev, "trace rsc has non zero reserved bytes\n");
		return -EINVAL;
	}

	trace = kzalloc(sizeof(*trace), GFP_KERNEL);
	if (!trace)
		return -ENOMEM;

	/* set the trace buffer dma properties */
	trace->trace_mem.len = rsc->len;
	trace->trace_mem.da = rsc->da;

	/* set pointer on rproc device */
	trace->rproc = rproc;

	/* make sure snprintf always null terminates, even if truncating */
	snprintf(name, sizeof(name), "aw_trace_%s", rsc->name);

	/* create the debugfs entry */
	trace->tfile = sunxi_rproc_create_aw_trace_file(name, rproc, trace);
	if (!trace->tfile) {
		kfree(trace);
		return -EINVAL;
	}

	list_add_tail(&trace->node, &rproc->traces);

	rproc->num_traces++;

	dev_info(dev, "add trace mem '%s', da: 0x%08x, len: %u\n", name, rsc->da, rsc->len);
	return 0;
}

static int sunxi_rproc_handle_rsc(struct rproc *rproc, u32 rsc_type,
			       void *ptr, int offset, int avail)
{
	dev_info(rproc->dev.parent, "handle vendor resource, type: %u\n", rsc_type);
	if (rsc_type == RSC_AW_TRACE) {
		return sunxi_rproc_handle_aw_trace(rproc, ptr, offset, avail);
	}

	return RSC_IGNORED;
}
#endif

static struct rproc_ops sunxi_rproc_ops = {
	.attach		= sunxi_rproc_attach,
	.start		= sunxi_rproc_start,
	.stop		= sunxi_rproc_stop,
	.da_to_va	= sunxi_rproc_da_to_va,
	.kick		= sunxi_rproc_kick,
	.parse_fw	= sunxi_rproc_parse_fw,
	.find_loaded_rsc_table = sunxi_rproc_rsc_table,
	.request_firmware = sunxi_rproc_request_firmware,
	.load		= sunxi_rproc_elf_load_segments,
	.get_boot_addr	= suxni_rproc_elf_get_boot_addr,
	.trigger_auto_boot = suxni_rproc_trigger_auto_boot,
#ifdef CONFIG_SUNXI_REMOTEPROC_TRACE_DEV
	.handle_rsc = sunxi_rproc_handle_rsc,
#endif
};

static const struct of_device_id sunxi_rproc_match[] = {
	{ .compatible = "allwinner,hifi4-rproc", .data = "hifi4" },
	{ .compatible = "allwinner,e906-rproc", .data = "e906" },
	{ .compatible = "allwinner,c906-rproc", .data = "c906" },
	{},
};
MODULE_DEVICE_TABLE(of, sunxi_rproc_match);

static int devm_sunxi_rproc_resource_get(struct rproc *rproc, struct platform_device *pdev)
{
	struct device *dev = rproc->dev.parent;
	struct sunxi_rproc *chip = rproc->priv;
	int ret;

	chip->rproc_priv = sunxi_rproc_priv_find(chip->name);
	if (!chip->rproc_priv) {
		dev_err(dev, "find rproc priv error\n");
		return -EINVAL;
	}

	ret = devm_sunxi_rproc_priv_resource_get(chip->rproc_priv, pdev);
	if (ret) {
		dev_err(dev, "resource get error\n");
		return ret;
	}

	return 0;
}

int sunxi_rproc_report_crash(const char *name, enum rproc_crash_type type)
{
	struct sunxi_rproc *chip, *tmp;

	list_for_each_entry_safe(chip, tmp, &sunxi_rproc_list, list) {
		if (!strcmp(chip->rproc->name, name)) {
			rproc_report_crash(chip->rproc, type);
			return 0;
		}
	}

	return -ENXIO;
}
EXPORT_SYMBOL(sunxi_rproc_report_crash);

static int sunxi_rproc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct of_device_id *of_id;
	struct sunxi_rproc *chip;
	struct device_node *np = dev->of_node;
	struct rproc *rproc;
	const char *fw_name = NULL;
	int ret;
	struct device_node *fw_np = NULL;

	dev_info(dev, "sunxi rproc driver %s\n", SUNXI_RPROC_VERSION);
	of_id = of_match_device(sunxi_rproc_match, dev);
	if (!of_id) {
		dev_err(dev, "No device of_id found\n");
		ret = -EINVAL;
		goto err_out;
	}

	/* we need to read firmware name at first. */
	ret = of_property_read_string(np, "firmware-name", &fw_name);
	if (ret < 0) {
		dev_info(dev, "failed to get firmware-name\n");
		fw_name = NULL;
	}

	rproc = rproc_alloc(dev, np->name, &sunxi_rproc_ops, fw_name, sizeof(*chip));
	if (!rproc) {
		ret = -ENOMEM;
		goto err_out;
	}

	rproc->has_iommu = false;
	rproc->auto_boot = of_property_read_bool(np, "auto-boot");
	chip = rproc->priv;
	chip->rproc = rproc;
	chip->name = (char *)of_id->data;

	ret = devm_sunxi_rproc_resource_get(rproc, pdev);
	if (ret) {
		dev_err(dev, "Failed to get resource\n");
		goto free_rproc;
	}

	chip->workqueue = create_workqueue(dev_name(dev));
	if (!chip->workqueue) {
		dev_err(dev, "Cannot create workqueue\n");
		ret = -ENOMEM;
		goto free_rproc;
	}

	platform_set_drvdata(pdev, rproc);

	ret = sunxi_rproc_request_mbox(rproc);
	if (ret) {
		dev_err(dev, "Request mbox failed\n");
		goto destroy_workqueue;
	}

	chip->is_booted = sunxi_rproc_priv_is_booted(chip->rproc_priv);
	if (chip->is_booted) {
		atomic_inc(&rproc->power);
		rproc->state = RPROC_EARLY_BOOT;
	}

	if ((rproc->state == RPROC_EARLY_BOOT) && rproc->auto_boot) {
		fw_np = of_parse_phandle(np, "fw-region", 0);
		if (fw_np) {
			struct resource r;
			resource_size_t len;

			ret = of_address_to_resource(fw_np, 0, &r);
			if (!ret) {
				len = resource_size(&r);

				dev_info(dev, "register memory firmware('%s') for '%s', addr: 0x%lx, size: %lx\n",
					rproc->firmware, chip->name, (unsigned long)r.start,  (unsigned long)len);
				ret = sunxi_register_memory_fw(rproc->firmware, r.start, len);
				if (ret < 0)
					dev_err(dev, "register memory firmware('%s') failed. ret: %d\n", rproc->firmware, ret);
			} else {
				dev_err(dev, "parse dt node '%s' failed, ret: %d\n", fw_np->full_name, ret);
			}
		}
	}

	ret = rproc_add(rproc);
	if (ret) {
		dev_err(dev, "Failed to register rproc\n");
		goto free_mbox;
	}

	list_add(&chip->list, &sunxi_rproc_list);

	dev_info(dev, "sunxi rproc driver probe ok\n");

	return ret;

free_mbox:
	sunxi_rproc_free_mbox(rproc);
destroy_workqueue:
	destroy_workqueue(chip->workqueue);
free_rproc:
	rproc_free(rproc);
err_out:
	return ret;
}

static int sunxi_rproc_remove(struct platform_device *pdev)
{
	struct rproc *rproc = platform_get_drvdata(pdev);
	struct sunxi_rproc *chip = rproc->priv;

	if (atomic_read(&rproc->power) > 0)
		rproc_shutdown(rproc);

	rproc_del(rproc);

	sunxi_rproc_free_mbox(rproc);

	destroy_workqueue(chip->workqueue);

	rproc_free(rproc);

	list_del(&chip->list);

	if (!list_empty(&sunxi_rproc_list))
		list_del(&sunxi_rproc_list);

	return 0;
}

static struct platform_driver sunxi_rproc_driver = {
	.probe = sunxi_rproc_probe,
	.remove = sunxi_rproc_remove,
	.driver = {
		.name = "sunxi-rproc", /* dev name */
		.of_match_table = sunxi_rproc_match,
	},
};
module_platform_driver(sunxi_rproc_driver);

MODULE_DESCRIPTION("Allwinnertech Remote Processor Control Driver");
MODULE_AUTHOR("wujiayi <wujiayi@allwinnertech.com>");
MODULE_AUTHOR("xuminghui <xuminghui@allwinnertech.com>");
MODULE_AUTHOR("caoyangguo <caoyangguo@allwinnertech.com>");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(SUNXI_RPROC_VERSION);
