export * from "./animation";
export * from "./clearance";
export * from "./codeExporter";
export * from "./draw";
export * from "./file";
export * from "./geometry";
export * from "./gifExporter";
export * from "./math";
export * from "./motionProfile";
export * from "./numberExpressions";
export * from "./pathMirroring";
export * from "./scrub";
export * from "./shapes";
export * from "./timeCalculator";
export * from "./directorySettings";
export * from "./sequence";
export * from "./variables";

export const DPI = 96 / 5;

export const titleCase = (str: string) =>
  `${str[0].toUpperCase()}${str.slice(1).toLowerCase()}`;
