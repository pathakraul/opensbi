/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Qualcomm, Inc
 *
 * Authors:
 *   Pawandeep Oza <pawandeep.oza@oss.qualcomm.com>
 */

#include <libfdt.h>
#include <stdbool.h>
#include <stddef.h>
#include <sbi/riscv_asm.h>
#include <sbi/sbi_byteorder.h>
#include <sbi/sbi_console.h>
#include <sbi/sbi_domain.h>
#include <sbi/sbi_domain_context.h>
#include <sbi/sbi_error.h>
#include <sbi/sbi_heap.h>
#include <sbi/sbi_list.h>
#include <sbi/sbi_mpxy.h>
#include <sbi/sbi_platform.h>
#include <sbi/sbi_scratch.h>
#include <sbi/sbi_string.h>
#include <sbi/sbi_types.h>
#include <sbi_utils/fdt/fdt_driver.h>
#include <sbi_utils/mailbox/rpmi_msgprot.h>

#define SBI_MPXY_BRIDGE_COMPATIBLE           "opensbi,mpxy-bridge"
#define SBI_MPXY_BRIDGE_PROP_SRC_DOMAIN_NAME "opensbi,source-domain-name"
#define SBI_MPXY_BRIDGE_PROP_DST_DOMAIN_NAME "opensbi,destination-domain-name"
#define SBI_MPXY_BRIDGE_PROP_SRC_CHAN        "opensbi,source-channel-id"
#define SBI_MPXY_BRIDGE_PROP_DST_CHAN        "opensbi,destination-channel-id"

#define SBI_MPXY_BRIDGE_DEFAULT_MSG_DATA_MAXLEN 4096
#define SBI_MPXY_BRIDGE_DOMAIN_NAME_MAXLEN      64

#define BRIDGE_FILL_FORWARD_RESP(resp_, status_, remaining_, returned_) \
	do { \
		(resp_)->status = (status_); \
		(resp_)->remaining = (remaining_); \
		(resp_)->returned = (returned_); \
	} while (0)

#define BRIDGE_FILL_COMPLETE_RESP(resp_, status_, messages_) \
	do { \
		(resp_)->status = (status_); \
		(resp_)->num_of_messages = (messages_); \
	} while (0)

struct mpxy_bridge_dst_ctx {
	void *respbuf;
	u32 resp_bufsize;
	unsigned long *resp_len_ptr;
	u32 start_index;
};

struct mpxy_bridge_src_ctx {
	bool occupied;
	bool retrieved;
	void *msgbuf;
	void *respbuf;
	u32 msg_len;
	u32 resp_bufsize;
	unsigned long *resp_len_ptr;
};

struct mpxy_bridge_hart_slot {
	int active_source;
	struct mpxy_bridge_src_ctx *src_ctx;
	struct mpxy_bridge_dst_ctx dst_resp;
};

struct mpxy_bridge_slot_store {
	struct mpxy_bridge_hart_slot *slots;
	u32 hart_count;
	u32 lost;
};

struct mpxy_bridge_source {
	char src_domain_name[SBI_MPXY_BRIDGE_DOMAIN_NAME_MAXLEN];
	u32 src_channel_id;
	struct sbi_domain *src_dom;
	struct sbi_mpxy_channel src_ch;
};

struct mpxy_bridge {
	struct sbi_dlist node;
	char dst_domain_name[SBI_MPXY_BRIDGE_DOMAIN_NAME_MAXLEN];
	u32 dst_channel_id;
	struct sbi_domain *dst_dom;
	struct sbi_mpxy_channel dst_ch;
	struct mpxy_bridge_source *sources;
	u32 source_count;
	struct mpxy_bridge_slot_store store;
};

static SBI_LIST_HEAD(mpxy_bridge_list);
static bool mpxy_bridge_notifier_registered;

static bool bridge_name_match(const char *a, const char *b)
{
	return a && b && !sbi_strcmp(a, b);
}

static int bridge_source_index(struct mpxy_bridge *br,
			       struct sbi_mpxy_channel *channel)
{
	u32 i;

	for (i = 0; i < br->source_count; i++)
		if (&br->sources[i].src_ch == channel)
			return (int)i;

	return -1;
}

static struct mpxy_bridge *bridge_from_channel(
					struct sbi_mpxy_channel *channel)
{
	struct mpxy_bridge *br;
	u32 i;

	sbi_list_for_each_entry(br, &mpxy_bridge_list, node) {
		if (&br->dst_ch == channel)
			return br;
		for (i = 0; i < br->source_count; i++)
			if (&br->sources[i].src_ch == channel)
				return br;
	}

	return NULL;
}

static int bridge_get_current_slot(struct mpxy_bridge *br,
				   struct mpxy_bridge_hart_slot **slot)
{
	u32 hartindex = current_hartindex();
	int ret = SBI_OK;

	if (hartindex >= br->store.hart_count)
		ret = SBI_ERR_INVALID_PARAM;
	else
		*slot = &br->store.slots[hartindex];

	return ret;
}

static void bridge_src_ctx_set(struct mpxy_bridge_src_ctx *src,
			       void *msgbuf, u32 msg_len,
			       void *respbuf, u32 resp_bufsize,
			       unsigned long *resp_len)
{
	src->msgbuf = msgbuf;
	src->msg_len = msg_len;
	src->respbuf = respbuf;
	src->resp_bufsize = resp_bufsize;
	src->resp_len_ptr = resp_len;
}

static void bridge_src_ctx_release(struct mpxy_bridge_src_ctx *src)
{
	if (src)
		*src = (struct mpxy_bridge_src_ctx){ 0 };
}

static void bridge_slot_release(struct mpxy_bridge *br,
				struct mpxy_bridge_hart_slot *slot)
{
	u32 i;

	if (!br || !slot)
		return;

	for (i = 0; i < br->source_count; i++)
		bridge_src_ctx_release(&slot->src_ctx[i]);

	slot->active_source = -1;
	slot->dst_resp = (struct mpxy_bridge_dst_ctx){ 0 };
}

static int bridge_src_ctx_occupy(struct mpxy_bridge *br,
				 struct mpxy_bridge_hart_slot *slot,
				 u32 source_index)
{
	struct mpxy_bridge_src_ctx *src = &slot->src_ctx[source_index];
	int ret = SBI_OK;

	if (src->occupied) {
		/*
		 * this should never happen, as speicifc source would be blocked
		 * till the reciever receievs the message and completes it.
		 */
		br->store.lost++;
		sbi_printf("%s: slot occupied, message lost for domain=%s\n",
				__func__, br->sources[source_index].src_domain_name);
		ret = SBI_ERR_FAILED;
	} else {
		src->occupied = true;
		src->retrieved = false;
	}

	return ret;
}

static int bridge_find_pending_source(struct mpxy_bridge *br,
				      struct mpxy_bridge_hart_slot *slot)
{
	u32 i;

	for (i = 0; i < br->source_count; i++)
		if (slot->src_ctx[i].occupied && !slot->src_ctx[i].retrieved)
			return (int)i;

	return -1;
}

static void bridge_switch_domain(struct mpxy_bridge *br,
				 struct sbi_mpxy_channel *channel)
{
	if (channel == &br->dst_ch)
		sbi_domain_context_exit();
	else
		sbi_domain_context_enter(br->dst_ch.owner_domain);
}

static void bridge_save_dst_ctx(struct mpxy_bridge_hart_slot *slot,
				void *respbuf, u32 resp_bufsize,
				unsigned long *resp_len, void *msgbuf)
{
	slot->dst_resp.respbuf = respbuf;
	slot->dst_resp.resp_bufsize = resp_bufsize;
	slot->dst_resp.resp_len_ptr = resp_len;
	slot->dst_resp.start_index = msgbuf ? ((u32 *)msgbuf)[0] : 0;
}

static void bridge_fill_forward_resp(struct mpxy_bridge_hart_slot *slot,
				     u32 source_index,
				     struct mpxy_bridge_src_ctx *src)
{
	struct rpmi_request_forward_resp *resp;
	u32 offset = slot->dst_resp.start_index;
	u32 returned_len;

	if (!slot->dst_resp.respbuf)
		return;

	returned_len = offset > src->msg_len ? 0 : src->msg_len - offset;
	resp = (struct rpmi_request_forward_resp *)slot->dst_resp.respbuf;

	if (slot->dst_resp.resp_bufsize < sizeof(*resp) + returned_len) {
		BRIDGE_FILL_FORWARD_RESP(resp, RPMI_ERR_INVALID_PARAM, 0, 0);
		if (slot->dst_resp.resp_len_ptr)
			*slot->dst_resp.resp_len_ptr = sizeof(*resp);
	} else {
		BRIDGE_FILL_FORWARD_RESP(resp, RPMI_SUCCESS, 0, returned_len);
		if (returned_len)
			sbi_memcpy(resp->data, (u8 *)src->msgbuf + offset, returned_len);
		if (slot->dst_resp.resp_len_ptr)
			*slot->dst_resp.resp_len_ptr = sizeof(*resp) + returned_len;
		src->retrieved = true;
		slot->active_source = (int)source_index;
		slot->dst_resp = (struct mpxy_bridge_dst_ctx){ 0 };
	}
}

static int bridge_src_send(struct mpxy_bridge *br,
			   struct mpxy_bridge_hart_slot *slot,
			   struct mpxy_bridge_src_ctx *src,
			   u32 source_index)
{
	int ret;

	ret = bridge_src_ctx_occupy(br, slot, source_index);
	if (!ret) {
		bridge_fill_forward_resp(slot, source_index, src);
		bridge_switch_domain(br, &br->sources[source_index].src_ch);
	}

	return ret;
}

static int bridge_build_retrieve_resp(struct mpxy_bridge *br,
				      struct mpxy_bridge_hart_slot *slot,
				      u32 source_index, void *respbuf,
				      u32 resp_bufsize,
				      unsigned long *resp_len)
{
	struct mpxy_bridge_src_ctx *src;
	struct rpmi_request_forward_resp *resp;
	int ret = SBI_OK;

	if (source_index >= br->source_count || !respbuf || !resp_len)
		ret = SBI_ERR_INVALID_PARAM;
	else {
		src = &slot->src_ctx[source_index];
		resp = (struct rpmi_request_forward_resp *)respbuf;
		if (!src->occupied || !src->msgbuf) {
			BRIDGE_FILL_FORWARD_RESP(resp, RPMI_ERR_NO_DATA, 0, 0);
			*resp_len = sizeof(*resp);
		} else if (resp_bufsize < sizeof(*resp) + src->msg_len) {
			BRIDGE_FILL_FORWARD_RESP(resp, RPMI_ERR_INVALID_PARAM, 0, 0);
			*resp_len = sizeof(*resp);
		} else {
			BRIDGE_FILL_FORWARD_RESP(resp, RPMI_SUCCESS, 0, src->msg_len);
			sbi_memcpy(resp->data, src->msgbuf, src->msg_len);
			*resp_len = sizeof(*resp) + src->msg_len;
			src->retrieved = true;
			slot->active_source = (int)source_index;
		}
	}

	return ret;
}

static int bridge_dst_retrieve(struct mpxy_bridge *br,
			       struct mpxy_bridge_hart_slot *slot,
			       void *msgbuf, u32 msg_len, void *respbuf,
			       u32 resp_bufsize, unsigned long *resp_len)
{
	int source_index = bridge_find_pending_source(br, slot);
	int ret = SBI_OK;

	(void)msg_len;

	if (source_index < 0) {
		bridge_save_dst_ctx(slot, respbuf, resp_bufsize, resp_len, msgbuf);
		bridge_switch_domain(br, &br->dst_ch);
	} else {
		ret = bridge_build_retrieve_resp(br, slot, (u32)source_index,
						 respbuf, resp_bufsize, resp_len);
	}

	return ret;
}

static int bridge_dst_complete(struct mpxy_bridge *br,
			       struct mpxy_bridge_hart_slot *slot,
			       void *msgbuf, u32 msg_len, void *respbuf,
			       u32 resp_max_len, unsigned long *resp_len)
{
	struct rpmi_request_complete_resp *resp;
	struct mpxy_bridge_src_ctx *src = NULL;
	int ret = SBI_OK;

	if (!respbuf || !resp_len || resp_max_len < sizeof(*resp))
		ret = SBI_ERR_INVALID_PARAM;
	else if (slot->active_source < 0 ||
		 (u32)slot->active_source >= br->source_count) {
		resp = (struct rpmi_request_complete_resp *)respbuf;
		BRIDGE_FILL_COMPLETE_RESP(resp, RPMI_ERR_NO_DATA, 0);
		*resp_len = sizeof(*resp);
	} else {
		src = &slot->src_ctx[slot->active_source];
		resp = (struct rpmi_request_complete_resp *)respbuf;
		if (!src->occupied || !src->retrieved) {
			BRIDGE_FILL_COMPLETE_RESP(resp, RPMI_ERR_NO_DATA, 0);
			*resp_len = sizeof(*resp);
		} else if (src->respbuf && msg_len > src->resp_bufsize) {
			BRIDGE_FILL_COMPLETE_RESP(resp, RPMI_ERR_INVALID_PARAM, 0);
			*resp_len = sizeof(*resp);
		} else {
			if (src->respbuf && msg_len)
				sbi_memcpy(src->respbuf, msgbuf, msg_len);
			if (src->resp_len_ptr)
				*src->resp_len_ptr = msg_len;
			BRIDGE_FILL_COMPLETE_RESP(resp, RPMI_SUCCESS, 0);
			*resp_len = sizeof(*resp);
			bridge_src_ctx_release(src);
			slot->active_source = -1;
			bridge_switch_domain(br, &br->dst_ch);
		}
	}

	return ret;
}

static int bridge_dst_send_message_with_response(
				struct mpxy_bridge *br,
				struct mpxy_bridge_hart_slot *slot, u32 msg_id,
				void *msgbuf, u32 msg_len, void *respbuf,
				u32 resp_bufsize, unsigned long *resp_len)
{
	int ret = SBI_ERR_NOT_SUPPORTED;

	if (msg_id == REQFWD_RETRIEVE_CURRENT_MESSAGE)
		ret = bridge_dst_retrieve(br, slot, msgbuf, msg_len,
					   respbuf, resp_bufsize, resp_len);
	else if (msg_id == REQFWD_COMPLETE_CURRENT_MESSAGE)
		ret = bridge_dst_complete(br, slot, msgbuf, msg_len,
					   respbuf, resp_bufsize, resp_len);

	return ret;
}

static int bridge_src_send_message_with_response(
				struct mpxy_bridge *br,
				struct mpxy_bridge_hart_slot *slot,
				struct sbi_mpxy_channel *channel,
				void *msgbuf, u32 msg_len, void *respbuf,
				u32 resp_bufsize, unsigned long *resp_len)
{
	struct mpxy_bridge_src_ctx *src;
	int source_index = bridge_source_index(br, channel);
	int ret = SBI_ERR_NOT_SUPPORTED;

	if (source_index >= 0) {
		src = &slot->src_ctx[source_index];
		bridge_src_ctx_set(src, msgbuf, msg_len, respbuf,
				   resp_bufsize, resp_len);
		ret = bridge_src_send(br, slot, src, (u32)source_index);
	}

	return ret;
}

static int bridge_send_message_with_response(
				struct sbi_mpxy_channel *channel, u32 msg_id,
				void *msgbuf, u32 msg_len, void *respbuf,
				u32 resp_bufsize, unsigned long *resp_len)
{
	struct mpxy_bridge *br = bridge_from_channel(channel);
	struct mpxy_bridge_hart_slot *slot = NULL;
	int ret = SBI_ERR_NOT_SUPPORTED;

	if (br) {
		ret = bridge_get_current_slot(br, &slot);
		if (!ret) {
			if (channel == &br->dst_ch)
				ret = bridge_dst_send_message_with_response(
					br, slot, msg_id, msgbuf, msg_len,
					respbuf, resp_bufsize, resp_len);
			else
				ret = bridge_src_send_message_with_response(
					br, slot, channel, msgbuf, msg_len,
					respbuf, resp_bufsize, resp_len);
		}
	}

	return ret;
}

static int bridge_post_message(struct sbi_mpxy_channel *channel,
			       u32 msg_id, void *msgbuf, u32 msg_len)
{
	struct mpxy_bridge *br = bridge_from_channel(channel);
	struct mpxy_bridge_hart_slot *slot;
	struct mpxy_bridge_src_ctx *src;
	u32 hartindex = current_hartindex();
	int source_index;
	int ret = SBI_OK;

	(void)msg_id;

	if (!br || !br->dst_dom)
		ret = SBI_ERR_NOT_SUPPORTED;
	else if (hartindex >= br->store.hart_count)
		ret = SBI_ERR_INVALID_PARAM;
	else {
		source_index = bridge_source_index(br, channel);
		if (source_index < 0)
			ret = SBI_ERR_NOT_SUPPORTED;
		else {
			ret = bridge_get_current_slot(br, &slot);
			if (!ret) {
				src = &slot->src_ctx[source_index];
				bridge_src_ctx_set(src, msgbuf, msg_len, NULL, 0, NULL);
				ret = bridge_src_send(br, slot, src, (u32)source_index);
			}
		}
	}

	return ret;
}

static int bridge_send_message_without_response(
				struct sbi_mpxy_channel *channel, u32 msg_id,
				void *msgbuf, u32 msg_len)
{
	return bridge_post_message(channel, msg_id, msgbuf, msg_len);
}

static void bridge_init_channel(struct sbi_mpxy_channel *channel,
				struct sbi_domain *dom, u32 channel_id)
{
	sbi_memset(channel, 0, sizeof(*channel));
	channel->owner_domain = dom;
	channel->channel_id = channel_id;
	channel->attrs.msg_data_maxlen =
		SBI_MPXY_BRIDGE_DEFAULT_MSG_DATA_MAXLEN;
	channel->attrs.msi_control = 0;
	channel->attrs.eventsstate_ctrl = 0;
	channel->send_message_with_response = bridge_send_message_with_response;
	channel->send_message_without_response =
		bridge_send_message_without_response;
}

static int bridge_register_source(struct mpxy_bridge *br,
				  u32 source_index, struct sbi_domain *dom)
{
	struct mpxy_bridge_source *src = &br->sources[source_index];
	int ret = SBI_OK;

	if (source_index >= br->source_count)
		ret = SBI_ERR_INVALID_PARAM;
	else if (!src->src_dom) {
		bridge_init_channel(&src->src_ch, dom, src->src_channel_id);
		ret = sbi_mpxy_register_channel(&src->src_ch);
		if (!ret)
			src->src_dom = dom;
	}

	return ret;
}

static int bridge_register_destination(struct mpxy_bridge *br,
				       struct sbi_domain *dom)
{
	int ret = SBI_OK;

	if (!br->dst_dom) {
		bridge_init_channel(&br->dst_ch, dom, br->dst_channel_id);
		ret = sbi_mpxy_register_channel(&br->dst_ch);
		if (!ret)
			br->dst_dom = dom;
	}

	return ret;
}

static int bridge_get_string_prop(const void *fdt, int nodeoff,
				  const char *propname, char *out, size_t outsz)
{
	const char *value;
	int len;
	int ret = SBI_OK;

	value = fdt_getprop(fdt, nodeoff, propname, &len);
	if (!value || len <= 0)
		ret = SBI_ENODEV;
	else {
		if ((size_t)len >= outsz)
			len = (int)outsz - 1;
		sbi_memcpy(out, value, len);
		out[len] = '\0';
	}

	return ret;
}

static int bridge_get_u32_prop(const void *fdt, int nodeoff,
				const char *propname, u32 *out)
{
	const fdt32_t *value;
	int len;
	int ret = SBI_OK;

	value = fdt_getprop(fdt, nodeoff, propname, &len);
	if (!value || len < (int)sizeof(*value))
		ret = SBI_ENODEV;
	else
		*out = fdt32_to_cpu(*value);

	return ret;
}

static bool sbi_hartmask_equal(const struct sbi_hartmask *a,
			       const struct sbi_hartmask *b)
{
	u32 hartindex;

	sbi_hartmask_for_each_hartindex(hartindex, a)
		if (!sbi_hartmask_test_hartindex(hartindex, b))
			return false;
	sbi_hartmask_for_each_hartindex(hartindex, b)
		if (!sbi_hartmask_test_hartindex(hartindex, a))
			return false;

	return true;
}

static void bridge_try_bind_domain(struct mpxy_bridge *br,
				   const struct sbi_domain *dom)
{
	u32 i;

	if (!br || !dom)
		return;

	for (i = 0; i < br->source_count; i++)
		if (!br->sources[i].src_dom &&
		    bridge_name_match(dom->name,
				      br->sources[i].src_domain_name))
			bridge_register_source(br, i, (struct sbi_domain *)dom);

	if (!br->dst_dom &&
	    bridge_name_match(dom->name, br->dst_domain_name))
		bridge_register_destination(br, (struct sbi_domain *)dom);

	if (br->dst_dom)
		for (i = 0; i < br->source_count; i++)
			if (br->sources[i].src_dom &&
			    !sbi_hartmask_equal(
				br->sources[i].src_dom->possible_harts,
				br->dst_dom->possible_harts))
				sbi_printf("%s: source[%u] and destination have "
					   "different possible harts\n", __func__, i);
}

static void bridge_domain_notifier(const struct sbi_domain *dom, void *priv)
{
	struct mpxy_bridge *br;

	(void)priv;
	sbi_list_for_each_entry(br, &mpxy_bridge_list, node)
		bridge_try_bind_domain(br, dom);
}

static int bridge_parse_fdt(const void *fdt, int nodeoff,
				    struct mpxy_bridge *br)
{
	int ret;

	ret = bridge_get_string_prop(fdt, nodeoff,
				     SBI_MPXY_BRIDGE_PROP_DST_DOMAIN_NAME,
				     br->dst_domain_name,
				     sizeof(br->dst_domain_name));
	if (ret)
		return ret;

	return bridge_get_u32_prop(fdt, nodeoff,
				   SBI_MPXY_BRIDGE_PROP_DST_CHAN,
				   &br->dst_channel_id);
}

static int bridge_count_sources(const void *fdt, int nodeoff)
{
	int child;
	int count = 0;

	fdt_for_each_subnode(child, fdt, nodeoff)
		count++;
	return count;
}

static int bridge_parse_sources(const void *fdt, int nodeoff,
				struct mpxy_bridge *br)
{
	struct mpxy_bridge_source *src;
	int child;
	u32 index = 0;
	int ret = SBI_OK;

	fdt_for_each_subnode(child, fdt, nodeoff) {
		src = &br->sources[index];
		ret = bridge_get_string_prop(fdt, child,
					     SBI_MPXY_BRIDGE_PROP_SRC_DOMAIN_NAME,
					     src->src_domain_name,
					     sizeof(src->src_domain_name));
		if (ret)
			break;
		ret = bridge_get_u32_prop(fdt, child,
					  SBI_MPXY_BRIDGE_PROP_SRC_CHAN,
					  &src->src_channel_id);
		if (ret)
			break;
		index++;
	}

	return ret;
}

static int bridge_validate(struct mpxy_bridge *br)
{
	u32 i, j;
	int ret = SBI_OK;

	if (!br->dst_domain_name[0] || !br->source_count)
		ret = SBI_EINVAL;

	for (i = 0; !ret && i < br->source_count; i++) {
		if (!br->sources[i].src_domain_name[0] ||
		    br->sources[i].src_channel_id == br->dst_channel_id)
			ret = SBI_EINVAL;
		for (j = i + 1; !ret && j < br->source_count; j++)
			if (br->sources[i].src_channel_id ==
			    br->sources[j].src_channel_id)
				ret = SBI_EINVAL;
	}

	return ret;
}

static void bridge_free(struct mpxy_bridge *br)
{
	u32 i;

	if (!br)
		return;

	for (i = 0; i < br->store.hart_count; i++) {
		if (br->store.slots[i].src_ctx)
			sbi_free(br->store.slots[i].src_ctx);
	}

	if (br->store.slots)
		sbi_free(br->store.slots);
	
	if (br->sources)
		sbi_free(br->sources);

	sbi_free(br);
}

static struct mpxy_bridge *bridge_alloc(u32 hart_count, u32 source_count)
{
	struct mpxy_bridge *br;
	u32 i;

	if (!hart_count || !source_count)
		return NULL;

	br = sbi_zalloc(sizeof(*br));
	if (!br)
		return NULL;

	br->sources = sbi_zalloc(sizeof(*br->sources) * source_count);
	br->store.slots = sbi_zalloc(sizeof(*br->store.slots) * hart_count);

	if (!br->sources || !br->store.slots) {
		bridge_free(br);
		return NULL;
	}

	for (i = 0; i < hart_count; i++) {
		br->store.slots[i].src_ctx = sbi_zalloc(
			 sizeof(struct mpxy_bridge_src_ctx) * source_count);
		if (!br->store.slots[i].src_ctx) {
			bridge_free(br);
			return NULL;
		}
	}

	br->source_count = source_count;
	br->store.hart_count = hart_count;

	return br;
}

static int bridge_register_notifier(struct mpxy_bridge *br)
{
	int ret = SBI_OK;

	if (!mpxy_bridge_notifier_registered) {
		ret = sbi_domain_register_notifier(bridge_domain_notifier, NULL);
		if (!ret)
			mpxy_bridge_notifier_registered = true;
	}
	return ret;
}

static int bridge_init(const void *fdt, int nodeoff,
		       const struct fdt_match *match)
{
	struct mpxy_bridge *br = NULL;
	struct sbi_scratch *scratch;
	u32 hart_count;
	u32 source_count;
	u32 i, j;
	int ret;

	(void)match;

	scratch = sbi_hartindex_to_scratch(current_hartindex());
	hart_count = sbi_platform_hart_count(
			sbi_platform_ptr(scratch));

	source_count = bridge_count_sources(fdt, nodeoff);
	br = bridge_alloc(hart_count, source_count);
	ret = br ? SBI_OK : SBI_ENOMEM;

	if (!ret)
		ret = bridge_parse_fdt(fdt, nodeoff, br);

	if (!ret)
		ret = bridge_parse_sources(fdt, nodeoff, br);

	if (!ret)
		ret = bridge_validate(br);

	if (!ret) {
		for (i = 0; i < hart_count; i++) {
			bridge_slot_release(br, &br->store.slots[i]);
			for (j = 0; j < source_count; j++)
				bridge_src_ctx_release(&br->store.slots[i].src_ctx[j]);
		}

		sbi_list_add_tail(&br->node, &mpxy_bridge_list);

		ret = bridge_register_notifier(br);

		if (ret)
			sbi_list_del(&br->node);
	}

	if (ret && br)
		bridge_free(br);

	return ret;
}

static const struct fdt_match bridge_match[] = {
	{ .compatible = SBI_MPXY_BRIDGE_COMPATIBLE },
	{ }
};

const struct fdt_driver fdt_mpxy_bridge = {
	.match_table = bridge_match,
	.init = bridge_init,
};
