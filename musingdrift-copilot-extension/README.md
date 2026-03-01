# Musingdrift Copilot Extension

这是一个 VS Code 插件示例，可通过命令面板调用 musingdrift API，并将结果插入到当前光标处。

## 使用方法

1. 在 VS Code 里按 `F5` 启动插件开发环境（或用 `npm install && npm run compile` 构建）。
2. 选中你想作为 prompt 的文本，或直接运行命令后输入 prompt。
3. 按 `Ctrl+Shift+P`，输入 `Ask Musingdrift AI` 并回车。
4. 插件会将 API 返回内容插入到光标处。

## 配置
- 修改 `src/extension.ts` 里的 `API_URL` 为你的实际 API endpoint。
- 如需调整请求体结构或响应解析方式，也请在 `extension.ts` 里修改。

## 依赖
- node-fetch
- vscode 扩展 API

---
如需进一步定制，请根据实际 API 文档调整。
