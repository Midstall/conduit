//! The interrupt-controller device-class contract. The seam that lets an `irq`
//! resource be acted on (PLIC/CLINT/GIC). On Ferrite userspace, the normalized
//! IRQ number is fed to SYS_IRQ_CREATE instead; this contract is for in-binary
//! controllers (Weir, or a Ferrite kernel driver).

const Intc = @This();

ctx: ?*anyopaque,
enable_fn: *const fn (ctx: ?*anyopaque, irq: u32) void,
disable_fn: *const fn (ctx: ?*anyopaque, irq: u32) void,
claim_fn: *const fn (ctx: ?*anyopaque) ?u32,
complete_fn: *const fn (ctx: ?*anyopaque, irq: u32) void,

pub fn enable(self: Intc, irq: u32) void {
    self.enable_fn(self.ctx, irq);
}

pub fn disable(self: Intc, irq: u32) void {
    self.disable_fn(self.ctx, irq);
}

/// Claim the highest-priority pending interrupt, if any.
pub fn claim(self: Intc) ?u32 {
    return self.claim_fn(self.ctx);
}

/// Signal end-of-interrupt for a claimed IRQ.
pub fn complete(self: Intc, irq: u32) void {
    self.complete_fn(self.ctx, irq);
}

pub fn from(impl: anytype) Intc {
    const P = @TypeOf(impl);
    const gen = struct {
        fn enable(ctx: ?*anyopaque, irq: u32) void {
            const self: P = @ptrCast(@alignCast(ctx));
            self.enable(irq);
        }
        fn disable(ctx: ?*anyopaque, irq: u32) void {
            const self: P = @ptrCast(@alignCast(ctx));
            self.disable(irq);
        }
        fn claim(ctx: ?*anyopaque) ?u32 {
            const self: P = @ptrCast(@alignCast(ctx));
            return self.claim();
        }
        fn complete(ctx: ?*anyopaque, irq: u32) void {
            const self: P = @ptrCast(@alignCast(ctx));
            self.complete(irq);
        }
    };
    return .{
        .ctx = impl,
        .enable_fn = gen.enable,
        .disable_fn = gen.disable,
        .claim_fn = gen.claim,
        .complete_fn = gen.complete,
    };
}
