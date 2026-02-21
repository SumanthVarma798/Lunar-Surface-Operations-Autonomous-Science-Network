const fs = require("fs");

function patchFile(filename) {
  let content = fs.readFileSync(filename, "utf8");

  // Replace Date.now() -> LSOASTime.now()
  content = content.replace(/Date\.now\(\)/g, "LSOASTime.now()");

  // Replace setInterval -> LSOASTime.setInterval (careful not to replace if it's already LSOASTime)
  content = content.replace(
    /(?<!LSOASTime\.)setInterval\(/g,
    "LSOASTime.setInterval(",
  );

  // Replace setTimeout -> LSOASTime.setTimeout
  content = content.replace(
    /(?<!LSOASTime\.)setTimeout\(/g,
    "LSOASTime.setTimeout(",
  );

  // Replace clearInterval -> LSOASTime.clearInterval
  content = content.replace(
    /(?<!LSOASTime\.)clearInterval\(/g,
    "LSOASTime.clearInterval(",
  );

  // Also replace clearTimeout if used
  content = content.replace(
    /(?<!LSOASTime\.)clearTimeout\(/g,
    "LSOASTime.clearTimeout(",
  );

  fs.writeFileSync(filename, content, "utf8");
  console.log("Patched " + filename);
}

patchFile("app.js");
patchFile("lunar3d.js");
