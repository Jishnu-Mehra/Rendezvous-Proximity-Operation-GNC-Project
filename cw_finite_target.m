function [u1,u2,info] = cw_finite_target(x0,xf,tb1,tc,tb2,A,B)

    assert(all([tb1,tc,tb2] >= 0))
    assert(tb1 > 0 && tb2 > 0)

    [F1,G1] = cw_step(A,B,tb1);
    [Fc,~] = cw_step(A,B,tc);
    [F2,G2] = cw_step(A,B,tb2);

    M = [F2*Fc*G1, G2];
    xfree = F2*Fc*F1*x0;

    U = M\(xf-xfree);

    u1 = U(1:3);
    u2 = U(4:6);

    info.cond = cond(M);
    info.residual = norm(M*U-(xf-xfree));
    info.xfinal = xfree+M*U;
    info.error = norm(info.xfinal-xf);
end

function [F,G] = cw_step(A,B,t)
    Z = expm([A B; zeros(3,9)]*t);
    F = Z(1:6,1:6);
    G = Z(1:6,7:9);
end