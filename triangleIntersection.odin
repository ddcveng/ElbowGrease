package mojsej

import "core:math/linalg"
import rl "vendor:raylib"

// Real Time collision detection 5.1.5
closest_point_on_triangle :: proc(p, a, b, c: rl.Vector3) -> rl.Vector3 {
    // Check if P in vertex region outside A
    ab := b - a
    ac := c - a
    ap := p - a
    d1 := linalg.dot(ab, ap)
    d2 := linalg.dot(ac, ap)
    if d1 <= 0.0 && d2 <= 0.0 do return a // barycentric coordinates (1,0,0)
    // Check if P in vertex region outside B
    bp := p - b
    d3 := linalg.dot(ab, bp)
    d4 := linalg.dot(ac, bp)
    if d3 >= 0.0 && d4 <= d3 do return b // barycentric coordinates (0,1,0)
    // Check if P in edge region of AB, if so return projection of P onto AB
    vc := d1 * d4 - d3 * d2
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        v := d1 / (d1 - d3)
        return a + v * ab // barycentric coordinates (1-v,v,0)
    }
    // Check if P in vertex region outside C
    cp := p - c
    d5 := linalg.dot(ab, cp)
    d6 := linalg.dot(ac, cp)
    if d6 >= 0.0 && d5 <= d6 do return c // barycentric coordinates (0,0,1)
    // Check if P in edge region of AC, if so return projection of P onto AC
    vb := d5 * d2 - d1 * d6
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        w := d2 / (d2 - d6)
        return a + w * ac // barycentric coordinates (1-w,0,w)
    }
    // Check if P in edge region of BC, if so return projection of P onto BC
    va := d3 * d6 - d5 * d4
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        w := (d4 - d3) / ((d4 - d3) + (d5 - d6))
        return b + w * (c - b) // barycentric coordinates (0,1-w,w)
    }
    // P inside face region. Compute Q through its barycentric coordinates (u,v,w)
    denom := 1.0 / (va + vb + vc)
    v := vb * denom
    w := vc * denom
    return a + ab * v + ac * w // = u*a + v*b + w*c, u = va * denom = 1.0-v-w
}
