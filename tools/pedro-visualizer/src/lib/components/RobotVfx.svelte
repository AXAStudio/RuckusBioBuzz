<script lang="ts">
  /**
   * The parts of a state effect drawn around the robot rather than on it:
   * flames streaming out of the front for fire, and a ground shadow that
   * shrinks as the robot leaves the TILES for roll and flip. The robot image
   * itself does the rolling and flipping (see spriteMotion).
   */
  import { fireParticles, fireSparks, type ActiveVfx } from "../../utils/robotStates";

  /** Robot centre on screen, px. */
  export let x: number;
  export let y: number;
  /** Screen rotation of the robot image, degrees (as the image is drawn). */
  export let heading: number;
  /** Robot length (forward) and width (across) on screen, px. */
  export let lengthPx: number;
  export let widthPx: number;
  export let vfx: ActiveVfx | null;
  /** 0-1 height above the TILES, from spriteMotion. */
  export let lift = 0;

  $: particles = vfx?.kind === "fire" ? fireParticles(vfx) : [];
  $: sparks = vfx?.kind === "fire" ? fireSparks(vfx) : [];

  /** White-hot at birth, through yellow and orange, to spent red. */
  function flameColor(heat: number, opacity: number): string {
    const stops: [number, [number, number, number]][] = [
      [0, [255, 244, 170]],
      [0.2, [255, 190, 40]],
      [0.5, [255, 96, 12]],
      [1, [170, 20, 8]],
    ];
    let a = stops[0];
    let b = stops[stops.length - 1];
    for (let i = 0; i < stops.length - 1; i++) {
      if (heat >= stops[i][0] && heat <= stops[i + 1][0]) {
        a = stops[i];
        b = stops[i + 1];
        break;
      }
    }
    const t = (heat - a[0]) / Math.max(1e-6, b[0] - a[0]);
    const mix = (i: number) => Math.round(a[1][i] + (b[1][i] - a[1][i]) * t);
    return `rgba(${mix(0)}, ${mix(1)}, ${mix(2)}, ${opacity.toFixed(3)})`;
  }

  $: glow = vfx?.kind === "fire"
    ? Math.min(1, vfx.elapsed / 0.12) * (0.75 + 0.25 * Math.sin(vfx.elapsed * 38))
    : 0;
</script>

{#if vfx && (vfx.kind === "roll" || vfx.kind === "flip")}
  <!-- Ground shadow: stays on the TILES while the robot goes up. -->
  <div
    class="absolute pointer-events-none z-[19] rounded-[40%]"
    style={`left: ${x}px; top: ${y}px; width: ${lengthPx}px; height: ${widthPx}px;
      transform: translate(-50%, -50%) rotate(${heading}deg) translate(${(lift * lengthPx * 0.12).toFixed(1)}px, ${(lift * widthPx * 0.18).toFixed(1)}px) scale(${(1 - 0.35 * lift).toFixed(3)});
      background: radial-gradient(ellipse at center, rgba(0,0,0,${(0.45 - 0.25 * lift).toFixed(3)}) 0%, rgba(0,0,0,0) 70%);`}
  ></div>
{/if}

{#if vfx && vfx.kind === "fire"}
  <!-- Rotates with the robot; +x is the robot's front. -->
  <div
    class="absolute pointer-events-none z-[21]"
    style={`left: ${x}px; top: ${y}px; width: 0; height: 0; transform: rotate(${heading}deg);`}
    aria-hidden="true"
  >
    <!-- Muzzle glow at the front edge -->
    <div
      class="absolute rounded-full"
      style={`left: ${lengthPx / 2}px; top: 0; width: ${widthPx * 0.9}px; height: ${widthPx * 0.9}px;
        transform: translate(-50%, -50%); mix-blend-mode: screen;
        background: radial-gradient(circle, rgba(255,240,200,${(0.9 * glow).toFixed(3)}) 0%, rgba(255,150,40,${(0.5 * glow).toFixed(3)}) 35%, rgba(255,80,0,0) 70%);`}
    ></div>
    {#each particles as particle}
      <div
        class="absolute rounded-full"
        style={`left: ${(lengthPx / 2 + particle.ahead * lengthPx).toFixed(1)}px;
          top: ${(particle.side * widthPx).toFixed(1)}px;
          width: ${(particle.size * widthPx).toFixed(1)}px; height: ${(particle.size * widthPx).toFixed(1)}px;
          transform: translate(-50%, -50%); filter: blur(${(0.5 + particle.heat * 1.5).toFixed(1)}px);
          background: radial-gradient(circle, ${flameColor(particle.heat * 0.5, particle.opacity)} 0%, ${flameColor(Math.min(1, particle.heat + 0.25), particle.opacity * 0.9)} 45%, rgba(0,0,0,0) 70%);`}
      ></div>
    {/each}
    {#each sparks as spark}
      <div
        class="absolute rounded-full"
        style={`left: ${(lengthPx / 2 + spark.ahead * lengthPx).toFixed(1)}px;
          top: ${(spark.side * widthPx).toFixed(1)}px;
          width: ${Math.max(2, spark.size * widthPx).toFixed(1)}px; height: ${Math.max(2, spark.size * widthPx).toFixed(1)}px;
          transform: translate(-50%, -50%); background: rgba(255, 236, 150, ${spark.opacity.toFixed(3)});
          box-shadow: 0 0 4px rgba(255, 150, 30, ${spark.opacity.toFixed(3)});`}
      ></div>
    {/each}
  </div>
{/if}
