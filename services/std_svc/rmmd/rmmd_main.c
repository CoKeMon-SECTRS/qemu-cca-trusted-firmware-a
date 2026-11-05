/*
 * Copyright (c) 2021-2024, Arm Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>
#include <errno.h>
#include <inttypes.h>
#include <stdint.h>
#include <string.h>

#include <arch_helpers.h>
#include <arch_features.h>
#include <bl31/bl31.h>
#include <common/debug.h>
#include <common/runtime_svc.h>
#include <context.h>
#include <lib/el3_runtime/context_mgmt.h>
#include <lib/el3_runtime/cpu_data.h>
#include <lib/el3_runtime/pubsub.h>
#include <lib/extensions/pmuv3.h>
#include <lib/extensions/sys_reg_trace.h>
#include <lib/gpt_rme/gpt_rme.h>

#include <lib/spinlock.h>
#include <lib/utils.h>
#include <lib/xlat_tables/xlat_tables_v2.h>
#include <plat/common/common_def.h>
#include <plat/common/platform.h>
#include <platform_def.h>
#include <services/rmmd_svc.h>
#include <smccc_helpers.h>
#include <lib/extensions/sme.h>
#include <lib/extensions/sve.h>
#include "arch/aarch32/arch_helpers.h"
#include "common/ep_info.h"
#include "lib/el3_runtime/context_el1.h"
#include "rmmd_initial_context.h"
#include "rmmd_private.h"

/*******************************************************************************
 * RMM boot failure flag
 ******************************************************************************/
static bool rmm_boot_failed;

/*******************************************************************************
 * OS currently running in the normal world
 ******************************************************************************/
static uint8_t first_linux_boot=1;
static uint8_t first_micro_switch=0;

#define SYSTEMREG 0
#define PAGETABLE 1

#define MICRO_W 0
#define LINUX_W 1

/*******************************************************************************
 * RMM context information.
 ******************************************************************************/
rmmd_rmm_context_t rmm_context[PLATFORM_CORE_COUNT];

/*******************************************************************************
 * RMM entry point information. Discovered on the primary core and reused
 * on secondary cores.
 ******************************************************************************/
static entry_point_info_t *rmm_ep_info;

/*******************************************************************************
 * Static function declaration.
 ******************************************************************************/
static int32_t rmm_init(void);

/*******************************************************************************
 * This function takes an RMM context pointer and performs a synchronous entry
 * into it.
 ******************************************************************************/
uint64_t rmmd_rmm_sync_entry(rmmd_rmm_context_t *rmm_ctx)
{
	uint64_t rc;

	assert(rmm_ctx != NULL);

	cm_set_context(&(rmm_ctx->cpu_ctx), REALM);

	/* Restore the realm context assigned above */
	cm_el2_sysregs_context_restore(REALM);
	cm_set_next_eret_context(REALM);

	/* Enter RMM */
	rc = rmmd_rmm_enter(&rmm_ctx->c_rt_ctx);

	/*
	 * Save realm context. EL2 Non-secure context will be restored
	 * before exiting Non-secure world, therefore there is no need
	 * to clear EL2 context registers.
	 */
	cm_el2_sysregs_context_save(REALM);

	return rc;
}

/*******************************************************************************
 * This function returns to the place where rmmd_rmm_sync_entry() was
 * called originally.
 ******************************************************************************/
__dead2 void rmmd_rmm_sync_exit(uint64_t rc)
{
	rmmd_rmm_context_t *ctx = &rmm_context[plat_my_core_pos()];

	/* Get context of the RMM in use by this CPU. */
	assert(cm_get_context(REALM) == &(ctx->cpu_ctx));

	/*
	 * The RMMD must have initiated the original request through a
	 * synchronous entry into RMM. Jump back to the original C runtime
	 * context with the value of rc in x0;
	 */
	rmmd_rmm_exit(ctx->c_rt_ctx, rc);

	panic();
}

static void rmm_el2_context_init(el2_sysregs_t *regs)
{
	write_el2_ctx_common(regs, spsr_el2, REALM_SPSR_EL2);
	write_el2_ctx_common(regs, sctlr_el2, SCTLR_EL2_RES1);
}

/*******************************************************************************
 * Enable architecture extensions on first entry to Realm world.
 ******************************************************************************/

static void manage_extensions_realm(cpu_context_t *ctx)
{
	pmuv3_enable(ctx);

	/*
	 * Enable access to TPIDR2_EL0 if SME/SME2 is enabled for Non Secure world.
	 */
	if (is_feat_sme_supported()) {
		sme_enable(ctx);
	}
}

static void manage_extensions_realm_per_world(void)
{
	cm_el3_arch_init_per_world(&per_world_context[CPU_CONTEXT_REALM]);

	if (is_feat_sve_supported()) {
	/*
	 * Enable SVE and FPU in realm context when it is enabled for NS.
	 * Realm manager must ensure that the SVE and FPU register
	 * contexts are properly managed.
	 */
		sve_enable_per_world(&per_world_context[CPU_CONTEXT_REALM]);
	}

	/* NS can access this but Realm shouldn't */
	if (is_feat_sys_reg_trace_supported()) {
		sys_reg_trace_disable_per_world(&per_world_context[CPU_CONTEXT_REALM]);
	}

	/*
	 * If SME/SME2 is supported and enabled for NS world, then disable trapping
	 * of SME instructions for Realm world. RMM will save/restore required
	 * registers that are shared with SVE/FPU so that Realm can use FPU or SVE.
	 */
	if (is_feat_sme_supported()) {
		sme_enable_per_world(&per_world_context[CPU_CONTEXT_REALM]);
	}
}

/*******************************************************************************
 * Jump to the RMM for the first time.
 ******************************************************************************/
static int32_t rmm_init(void)
{
	long rc;
	rmmd_rmm_context_t *ctx = &rmm_context[plat_my_core_pos()];

	INFO("RMM init start.\n");

	/* Enable architecture extensions */
	manage_extensions_realm(&ctx->cpu_ctx);

	manage_extensions_realm_per_world();

	/* Initialize RMM EL2 context. */
	rmm_el2_context_init(&ctx->cpu_ctx.el2_sysregs_ctx);

	rc = rmmd_rmm_sync_entry(ctx);
	if (rc != E_RMM_BOOT_SUCCESS) {
		ERROR("RMM init failed: %ld\n", rc);
		/* Mark the boot as failed for all the CPUs */
		rmm_boot_failed = true;
		return 0;
	}

	INFO("RMM init end.\n");

	return 1;
}

/*******************************************************************************
 * Load and read RMM manifest, setup RMM.
 ******************************************************************************/
int rmmd_setup(void)
{
	size_t shared_buf_size __unused;
	uintptr_t shared_buf_base;
	uint32_t ep_attr;
	unsigned int linear_id = plat_my_core_pos();
	rmmd_rmm_context_t *rmm_ctx = &rmm_context[linear_id];
	struct rmm_manifest *manifest;
	int rc;

	/* Make sure RME is supported. */
	if (is_feat_rme_present() == 0U) {
		/* Mark the RMM boot as failed for all the CPUs */
		rmm_boot_failed = true;
		return -ENOTSUP;
	}

	rmm_ep_info = bl31_plat_get_next_image_ep_info(REALM);
	if ((rmm_ep_info == NULL) || (rmm_ep_info->pc == 0)) {
		WARN("No RMM image provided by BL2 boot loader, Booting "
		     "device without RMM initialization. SMCs destined for "
		     "RMM will return SMC_UNK\n");

		/* Mark the boot as failed for all the CPUs */
		rmm_boot_failed = true;
		return -ENOENT;
	}

	/* Initialise an entrypoint to set up the CPU context */
	ep_attr = EP_REALM;
	if ((read_sctlr_el3() & SCTLR_EE_BIT) != 0U) {
		ep_attr |= EP_EE_BIG;
	}

	SET_PARAM_HEAD(rmm_ep_info, PARAM_EP, VERSION_1, ep_attr);
	rmm_ep_info->spsr = SPSR_64(MODE_EL2,
					MODE_SP_ELX,
					DISABLE_ALL_EXCEPTIONS);

	shared_buf_size =
			plat_rmmd_get_el3_rmm_shared_mem(&shared_buf_base);

	assert((shared_buf_size == SZ_4K) &&
					((void *)shared_buf_base != NULL));

	/* Zero out and load the boot manifest at the beginning of the share area */
	manifest = (struct rmm_manifest *)shared_buf_base;
	(void)memset((void *)manifest, 0, sizeof(struct rmm_manifest));

	rc = plat_rmmd_load_manifest(manifest);
	if (rc != 0) {
		ERROR("Error loading RMM Boot Manifest (%i)\n", rc);
		/* Mark the boot as failed for all the CPUs */
		rmm_boot_failed = true;
		return rc;
	}
	flush_dcache_range((uintptr_t)shared_buf_base, shared_buf_size);

	/*
	 * Prepare coldboot arguments for RMM:
	 * arg0: This CPUID (primary processor).
	 * arg1: Version for this Boot Interface.
	 * arg2: PLATFORM_CORE_COUNT.
	 * arg3: Base address for the EL3 <-> RMM shared area. The boot
	 *       manifest will be stored at the beginning of this area.
	 */
	rmm_ep_info->args.arg0 = linear_id;
	rmm_ep_info->args.arg1 = RMM_EL3_INTERFACE_VERSION;
	rmm_ep_info->args.arg2 = PLATFORM_CORE_COUNT;
	rmm_ep_info->args.arg3 = shared_buf_base;

	/* Initialise RMM context with this entry point information */
	cm_setup_context(&rmm_ctx->cpu_ctx, rmm_ep_info);

	INFO("RMM setup done.\n");

	/* Register init function for deferred init.  */
	bl31_register_rmm_init(&rmm_init);

	return 0;
}

/*******************************************************************************
 * Forward SMC to the other security state
 ******************************************************************************/
static uint64_t	rmmd_smc_forward(uint32_t src_sec_state,
				 uint32_t dst_sec_state, uint64_t x0,
				 uint64_t x1, uint64_t x2, uint64_t x3,
				 uint64_t x4, void *handle)
{
	cpu_context_t *ctx = cm_get_context(dst_sec_state);

	/* Save incoming security state */
	cm_el2_sysregs_context_save(src_sec_state);

	/* Restore outgoing security state */
	cm_el2_sysregs_context_restore(dst_sec_state);
	cm_set_next_eret_context(dst_sec_state);

	/*
	 * As per SMCCCv1.2, we need to preserve x4 to x7 unless
	 * being used as return args. Hence we differentiate the
	 * onward and backward path. Support upto 8 args in the
	 * onward path and 4 args in return path.
	 * Register x4 will be preserved by RMM in case it is not
	 * used in return path.
	 */
	if (src_sec_state == NON_SECURE) {
		SMC_RET8(ctx, x0, x1, x2, x3, x4,
			 SMC_GET_GP(handle, CTX_GPREG_X5),
			 SMC_GET_GP(handle, CTX_GPREG_X6),
			 SMC_GET_GP(handle, CTX_GPREG_X7));
	}

	SMC_RET5(ctx, x0, x1, x2, x3, x4);
}

static cpu_context_t normal_world_contexts[PLATFORM_CORE_COUNT];
static el1_sysregs_t normal_world_el1_regs[2*PLATFORM_CORE_COUNT];

void initial_save_ctx(uint8_t ns_world)
{
	for(int i=0;i<PLATFORM_CORE_COUNT;i++){
		el1_sysregs_context_save(&normal_world_el1_regs[i+PLATFORM_CORE_COUNT*ns_world]);
		if(ns_world==MICRO_W){
			cpu_context_t *ctx_normal_world = get_cpu_data(cpu_context[get_cpu_context_index(NON_SECURE)]);
			memcpy(&normal_world_contexts[i],ctx_normal_world,sizeof(cpu_context_t));
		}
	}
}
void save_el1_sysregs(uint8_t ns_world)
{
	uint64_t cpuid = plat_my_core_pos();
	el1_sysregs_context_save(&normal_world_el1_regs[cpuid+PLATFORM_CORE_COUNT*ns_world]);
}
void restore_el1_sysregs(uint8_t ns_world)
{
	uint64_t cpuid = plat_my_core_pos();
	el1_sysregs_context_restore(&normal_world_el1_regs[cpuid+PLATFORM_CORE_COUNT*ns_world]);
}

void *get_current_micro_ctx(){
	uint64_t cpuid = plat_my_core_pos();
	return &normal_world_contexts[cpuid];
}

void flush_tlb(){
	tlbipaallos();
	dsb();
	isb();
}

uint64_t initial_linux_switch(void *handle){
	uint32_t image_type = NON_SECURE;
	entry_point_info_t *next_image_info;
	/* Save incoming security state */
	cm_el2_sysregs_context_save(NON_SECURE);
	initial_save_ctx(MICRO_W);

	next_image_info = bl31_plat_get_next_image_ep_info(image_type);
	// Set pointer to the start of the other OS (Linux here)
	next_image_info->pc=(uintptr_t)0x80000000;

	INFO("BL31: Preparing for EL3 exit to %s world\n",
		(image_type == SECURE) ? "secure" : "normal");
	print_entry_point_info(next_image_info);
	cm_init_my_context(next_image_info); // Initiliaze fresh new context for the second OS
	INFO("We are going to exit TF-A into the image at %lx\n",next_image_info->pc);

	gpt_enable_gpt1();

	cm_prepare_el3_exit_ns();

	first_linux_boot=0;
	first_micro_switch=1;

	flush_tlb();
	SMC_RET0(handle);
}

/*******************************************************************************
 * This function handles all SMCs in the range reserved for RMI. Each call is
 * either forwarded to the other security state or handled by the RMM dispatcher
 ******************************************************************************/
#define write_el2_ctx_common(ctx, reg, val)	((((ctx)->common).reg)	\
							= (uint64_t) (val))
uint64_t rmmd_rmi_handler(uint32_t smc_fid, uint64_t x1, uint64_t x2,
			  uint64_t x3, uint64_t x4, void *cookie,
			  void *handle, uint64_t flags)
{
	uint32_t src_sec_state;

	/* If RMM failed to boot, treat any RMI SMC as unknown */
	if (rmm_boot_failed) {
		WARN("RMMD: Failed to boot up RMM. Ignoring RMI call\n");
		SMC_RET1(handle, SMC_UNK);
	}

	/* Determine which security state this SMC originated from */
	src_sec_state = caller_sec_state(flags);

	/* RMI must not be invoked by the Secure world */
	if (src_sec_state == SMC_FROM_SECURE) {
		WARN("RMMD: RMI invoked by secure world.\n");
		SMC_RET1(handle, SMC_UNK);
	}

	if (src_sec_state == SMC_FROM_NON_SECURE) {
		if(smc_fid == (uint32_t) 0xc4000172){
			INFO("Call into the TF-A from a thread");
			SMC_RET0(handle);
		}
		unsigned long esr_el3_iss = read_esr_el3() & 0xffff;
		if(esr_el3_iss != 0 || smc_fid == (uint32_t) 0xc4000170 || smc_fid == (uint32_t) 0xc4000171) { // SMC_RMI_OS_SWITCH
			if (first_linux_boot){
				INFO("First Switch to Linux\n");
				return initial_linux_switch(handle);
			}else if(first_micro_switch){
				INFO("First Switch to the Microkernel\n");
				initial_save_ctx(LINUX_W);
				first_micro_switch=0;
			}

			cpu_context_t *ctx;
			if(smc_fid == (uint32_t) 0xc4000170 || esr_el3_iss != 0){
				VERBOSE("Switching to the Microkernel\n");

				ctx = cm_get_context(NON_SECURE);
				cm_el2_sysregs_context_save(NON_SECURE);
				save_el1_sysregs(LINUX_W);
				restore_el1_sysregs(MICRO_W);

				gpt_switch_gptbr();

				ctx = get_current_micro_ctx();
				cm_el2_sysregs_context_restore_helper(ctx);
				cm_set_next_context(ctx);
				handle=ctx;
			}else if (smc_fid == (uint32_t)0xc4000171) {
				VERBOSE("Switching to Linux\n");

				ctx = get_current_micro_ctx();
				cm_el2_sysregs_context_save_helper(ctx);
				save_el1_sysregs(MICRO_W);
				restore_el1_sysregs(LINUX_W);

				gpt_switch_gptbr_gpt1();

				ctx = cm_get_context(NON_SECURE);
				if(x3==SYSTEMREG){
					el2_sysregs_t *el2_sysregs_ctx = get_el2_sysregs_ctx(ctx);
					if(x2==5){
						write_el2_ctx_vhe(el2_sysregs_ctx, ttbr1_el2, x1);
					}else if(x2==4){
						write_el2_ctx_common(el2_sysregs_ctx, ttbr0_el2, x1);
					} else{
						ERROR("ERROR: Not matching any of the writes was not planned\n");
						SMC_RET0(handle);
					}
				}
				cm_el2_sysregs_context_restore_helper(ctx);
				cm_set_next_context(ctx);
				handle=ctx;
			}

			flush_tlb();

			if(smc_fid == (uint32_t) 0xc4000171){
				SMC_RET0(handle);
			}else if(esr_el3_iss != 0){
				uint8_t type = (esr_el3_iss & 0x0f00) >> 8;
				uint8_t reg = esr_el3_iss & 0x00ff;
				uint64_t value_reg = 0xff;
				switch(reg){
					case 0:
						value_reg=smc_fid;
						break;
					case 1:
						value_reg=x1;
						break;
					case 2:
						value_reg=x2;
						break;
					case 3:
						value_reg=x3;
						break;
					case 4:
						value_reg=x4;
						break;
					default:
						ERROR("Register is not among the 5 registers we have access to. We need to access the other registers saved on the stack. Request type is %d with register %d\n",type,reg);
						SMC_RET0(handle);
				}
				SMC_RET3(handle,value_reg,type, SYSTEMREG);
			}else if(x3==PAGETABLE){
				SMC_RET3(handle, x1 & 0xFFFFFFFF, x2, PAGETABLE);
			}else{
				SMC_RET3(handle, x1 >> 32, x1 & 0xFFFFFFFF, x2);
			}
		}
		/*
		 * If SVE hint bit is set in the flags then update the SMC
		 * function id and pass it on to the lower EL.
		 */
		if (is_sve_hint_set(flags)) {
			smc_fid |= (FUNCID_SVE_HINT_MASK <<
				    FUNCID_SVE_HINT_SHIFT);
		}
		/*
		 * Forward an RMI call from the Normal world to the Realm world as it
		 * is.
		 */
		VERBOSE("RMMD: RMI call from non-secure world.\n");
		return rmmd_smc_forward(NON_SECURE, REALM, smc_fid,
					x1, x2, x3, x4, handle);
	}

	if (src_sec_state != SMC_FROM_REALM) {
		SMC_RET1(handle, SMC_UNK);
	}

	switch (smc_fid) {
	case RMM_RMI_REQ_COMPLETE: {
		uint64_t x5 = SMC_GET_GP(handle, CTX_GPREG_X5);

		return rmmd_smc_forward(REALM, NON_SECURE, x1,
					x2, x3, x4, x5, handle);
	}
	default:
		WARN("RMMD: Unsupported RMM call 0x%08x\n", smc_fid);
		SMC_RET1(handle, SMC_UNK);
	}
}

/*******************************************************************************
 * This cpu has been turned on. Enter RMM to initialise R-EL2.  Entry into RMM
 * is done after initialising minimal architectural state that guarantees safe
 * execution.
 ******************************************************************************/
static void *rmmd_cpu_on_finish_handler(const void *arg)
{
	long rc;
	uint32_t linear_id = plat_my_core_pos();
	rmmd_rmm_context_t *ctx = &rmm_context[linear_id];

	if (rmm_boot_failed) {
		/* RMM Boot failed on a previous CPU. Abort. */
		ERROR("RMM Failed to initialize. Ignoring for CPU%d\n",
								linear_id);
		return NULL;
	}

	/*
	 * Prepare warmboot arguments for RMM:
	 * arg0: This CPUID.
	 * arg1 to arg3: Not used.
	 */
	rmm_ep_info->args.arg0 = linear_id;
	rmm_ep_info->args.arg1 = 0ULL;
	rmm_ep_info->args.arg2 = 0ULL;
	rmm_ep_info->args.arg3 = 0ULL;

	/* Initialise RMM context with this entry point information */
	cm_setup_context(&ctx->cpu_ctx, rmm_ep_info);

	/* Enable architecture extensions */
	manage_extensions_realm(&ctx->cpu_ctx);

	/* Initialize RMM EL2 context. */
	rmm_el2_context_init(&ctx->cpu_ctx.el2_sysregs_ctx);

	rc = rmmd_rmm_sync_entry(ctx);

	if (rc != E_RMM_BOOT_SUCCESS) {
		ERROR("RMM init failed on CPU%d: %ld\n", linear_id, rc);
		/* Mark the boot as failed for any other booting CPU */
		rmm_boot_failed = true;
	}

	return NULL;
}

/* Subscribe to PSCI CPU on to initialize RMM on secondary */
SUBSCRIBE_TO_EVENT(psci_cpu_on_finish, rmmd_cpu_on_finish_handler);

/* Convert GPT lib error to RMMD GTS error */
static int gpt_to_gts_error(int error, uint32_t smc_fid, uint64_t address)
{
	int ret;

	if (error == 0) {
		return E_RMM_OK;
	}

	if (error == -EINVAL) {
		ret = E_RMM_BAD_ADDR;
	} else {
		/* This is the only other error code we expect */
		assert(error == -EPERM);
		ret = E_RMM_BAD_PAS;
	}

	ERROR("RMMD: PAS Transition failed. GPT ret = %d, PA: 0x%"PRIx64 ", FID = 0x%x\n",
				error, address, smc_fid);
	return ret;
}

static int rmm_el3_ifc_get_feat_register(uint64_t feat_reg_idx,
					 uint64_t *feat_reg)
{
	if (feat_reg_idx != RMM_EL3_FEAT_REG_0_IDX) {
		ERROR("RMMD: Failed to get feature register %ld\n", feat_reg_idx);
		return E_RMM_INVAL;
	}

	*feat_reg = 0UL;
#if RMMD_ENABLE_EL3_TOKEN_SIGN
	*feat_reg |= RMM_EL3_FEAT_REG_0_EL3_TOKEN_SIGN_MASK;
#endif
	return E_RMM_OK;
}

/*******************************************************************************
 * This function handles RMM-EL3 interface SMCs
 ******************************************************************************/
uint64_t rmmd_rmm_el3_handler(uint32_t smc_fid, uint64_t x1, uint64_t x2,
				uint64_t x3, uint64_t x4, void *cookie,
				void *handle, uint64_t flags)
{
	uint64_t remaining_len = 0UL;
	uint32_t src_sec_state;
	int ret;

	/* If RMM failed to boot, treat any RMM-EL3 interface SMC as unknown */
	if (rmm_boot_failed) {
		WARN("RMMD: Failed to boot up RMM. Ignoring RMM-EL3 call\n");
		SMC_RET1(handle, SMC_UNK);
	}

	/* Determine which security state this SMC originated from */
	src_sec_state = caller_sec_state(flags);

	if (src_sec_state != SMC_FROM_REALM) {
		WARN("RMMD: RMM-EL3 call originated from secure or normal world\n");
		SMC_RET1(handle, SMC_UNK);
	}

	switch (smc_fid) {
	case RMM_GTSI_DELEGATE:
		ret = gpt_delegate_pas(x1, PAGE_SIZE_4KB, SMC_FROM_REALM);
		SMC_RET1(handle, gpt_to_gts_error(ret, smc_fid, x1));
	case RMM_GTSI_UNDELEGATE:
		ret = gpt_undelegate_pas(x1, PAGE_SIZE_4KB, SMC_FROM_REALM);
		SMC_RET1(handle, gpt_to_gts_error(ret, smc_fid, x1));
	case RMM_ATTEST_GET_PLAT_TOKEN:
		ret = rmmd_attest_get_platform_token(x1, &x2, x3, &remaining_len);
		SMC_RET3(handle, ret, x2, remaining_len);
	case RMM_ATTEST_GET_REALM_KEY:
		ret = rmmd_attest_get_signing_key(x1, &x2, x3);
		SMC_RET2(handle, ret, x2);
	case RMM_EL3_FEATURES:
		ret = rmm_el3_ifc_get_feat_register(x1, &x2);
		SMC_RET2(handle, ret, x2);
#if RMMD_ENABLE_EL3_TOKEN_SIGN
	case RMM_EL3_TOKEN_SIGN:
		return rmmd_el3_token_sign(handle, x1, x2, x3, x4);
#endif
	case RMM_BOOT_COMPLETE:
		VERBOSE("RMMD: running rmmd_rmm_sync_exit\n");
		rmmd_rmm_sync_exit(x1);

	default:
		WARN("RMMD: Unsupported RMM-EL3 call 0x%08x\n", smc_fid);
		SMC_RET1(handle, SMC_UNK);
	}
}
