<script lang="ts">
  /**
   * The parts of a state effect drawn around the robot rather than on it: the
   * volley for shoot - aim line, reticle on the goal, muzzle flash, NECTAR in
   * the air and the landing ring - and a ground shadow that shrinks as the robot
   * leaves the TILES for roll and flip. The robot image itself does the rolling,
   * flipping and recoil (see spriteMotion and App).
   */
  import type { ActiveVfx, ShotFrame } from "../../utils/robotStates";

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
  /** The volley at this moment, when the effect is shoot. */
  export let shot: ShotFrame | null = null;
  /** NECTAR diameter on screen, px. */
  export let ballPx = 8;
  /** Colour of the goal's alliance, for the balls, aim line and reticle. */
  export let color = "#facc15";

  const uid = Math.random().toString(36).slice(2, 8);

  $: elapsed = vfx?.elapsed ?? 0;
  $: muzzle = shot
    ? {
        x: shot.from.x + Math.cos((shot.angle * Math.PI) / 180) * lengthPx * 0.5,
        y: shot.from.y + Math.sin((shot.angle * Math.PI) / 180) * lengthPx * 0.5,
      }
    : null;
  $: reticleR = Math.max(10, ballPx * 2.2);
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

{#if vfx && vfx.kind === "shoot" && shot && muzzle}
  <svg
    class="absolute pointer-events-none z-[21]"
    style="left: 0; top: 0; width: 1px; height: 1px; overflow: visible;"
    aria-hidden="true"
  >
    <defs>
      <radialGradient id="nectar-{uid}" cx="35%" cy="35%" r="70%">
        <stop offset="0%" stop-color="#ffffff" stop-opacity="0.9" />
        <stop offset="35%" stop-color={color} />
        <stop offset="100%" stop-color={color} stop-opacity="0.85" />
      </radialGradient>
      <radialGradient id="flash-{uid}">
        <stop offset="0%" stop-color="#fffbe6" />
        <stop offset="40%" stop-color="#fde047" stop-opacity="0.9" />
        <stop offset="100%" stop-color="#f97316" stop-opacity="0" />
      </radialGradient>
    </defs>

    <!-- Aim line, marching toward the goal -->
    <line
      x1={muzzle.x}
      y1={muzzle.y}
      x2={shot.to.x}
      y2={shot.to.y}
      stroke={color}
      stroke-width="2"
      stroke-linecap="round"
      stroke-dasharray="6 6"
      stroke-dashoffset={(-elapsed * 60).toFixed(1)}
      opacity={(0.75 * shot.aim).toFixed(3)}
    />

    <!-- Reticle on the goal -->
    <g transform="translate({shot.to.x.toFixed(1)} {shot.to.y.toFixed(1)})" opacity={shot.aim.toFixed(3)}>
      <circle r={reticleR} fill="none" stroke="#0b0b0b" stroke-opacity="0.5" stroke-width="4" />
      <circle r={reticleR} fill="none" stroke={color} stroke-width="2" />
      <g transform="rotate({(elapsed * 120).toFixed(1)})">
        {#each [0, 90, 180, 270] as tick}
          <line
            x1={reticleR * 0.55}
            x2={reticleR * 1.35}
            y1="0"
            y2="0"
            stroke={color}
            stroke-width="2"
            stroke-linecap="round"
            transform="rotate({tick})"
          />
        {/each}
      </g>
      <circle r={(reticleR * (0.22 + 0.08 * Math.sin(elapsed * 14))).toFixed(2)} fill={color} />
    </g>

    <!-- Landing rings -->
    {#each shot.impacts as p}
      <circle
        cx={shot.to.x}
        cy={shot.to.y}
        r={(ballPx * (0.6 + 2.6 * p)).toFixed(2)}
        fill="none"
        stroke={color}
        stroke-width={(3 * (1 - p) + 0.5).toFixed(2)}
        opacity={(1 - p).toFixed(3)}
      />
      <circle cx={shot.to.x} cy={shot.to.y} r={(ballPx * 0.9 * (1 - p)).toFixed(2)} fill="#fffbe6" opacity={(0.8 * (1 - p)).toFixed(3)} />
    {/each}

    <!-- NECTAR in the air: shadow on the TILES, ball up the arc -->
    {#each shot.balls as ball}
      <ellipse
        cx={ball.x + ball.lift * ballPx * 0.6}
        cy={ball.y + ball.lift * ballPx * 0.9}
        rx={(ballPx * 0.5 * (1 - 0.35 * ball.lift)).toFixed(2)}
        ry={(ballPx * 0.4 * (1 - 0.35 * ball.lift)).toFixed(2)}
        fill="#000"
        opacity={(0.4 - 0.2 * ball.lift).toFixed(3)}
      />
      <circle
        cx={ball.x}
        cy={ball.y - ball.lift * ballPx * 1.2}
        r={(ballPx * 0.5 * (1 + 0.7 * ball.lift)).toFixed(2)}
        fill="url(#nectar-{uid})"
        stroke="#111"
        stroke-opacity="0.6"
        stroke-width="1"
      />
    {/each}

    <!-- Muzzle flash -->
    {#if shot.flash > 0}
      <g transform="translate({muzzle.x.toFixed(1)} {muzzle.y.toFixed(1)}) rotate({shot.angle.toFixed(1)})">
        <ellipse
          cx={ballPx * 0.9 * shot.flash}
          cy="0"
          rx={(ballPx * 1.9 * shot.flash).toFixed(2)}
          ry={(ballPx * 1.1 * shot.flash).toFixed(2)}
          fill="url(#flash-{uid})"
        />
      </g>
    {/if}
  </svg>
{/if}
