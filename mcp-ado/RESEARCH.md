# MCP Server for Azure DevOps - Research & Approaches

This document summarizes approaches to create a local MCP (Model Context Protocol) server that interfaces with Azure DevOps (git repos, work items, pull requests, pipelines, etc.).

## Executive Summary

There are **three main approaches** to get an MCP server for Azure DevOps:

| Approach | Effort | Recommended For |
|----------|--------|-----------------|
| 1. Use Microsoft's Official MCP Server | Minimal | Production use, full feature set |
| 2. Use Community MCP Server | Minimal | Alternative auth options, customization base |
| 3. Build Custom MCP Server | High | Specialized needs, learning MCP |

**Recommendation**: Use Microsoft's official Azure DevOps MCP Server unless you have specific requirements it doesn't meet.

---

## Approach 1: Microsoft's Official Azure DevOps MCP Server (Recommended)

### Overview
Microsoft released an official Azure DevOps MCP Server (GA as of October 2025) that runs locally and provides secure access to Azure DevOps resources.

### Key Features
- **Runs locally** - Sensitive project data never leaves your network
- **Browser-based authentication** - Uses Microsoft account login
- **Comprehensive coverage** - Projects, work items, PRs, builds, test plans, wiki, pipelines
- **Domain filtering** - Load only the tool groups you need
- **Free to use** - Only standard Azure DevOps pricing applies

### Installation

```bash
# Via npx (no installation needed)
npx -y @azure-devops/mcp <organization-name>

# Or install globally
npm install -g @azure-devops/mcp
```

### Configuration for VS Code

Create `.vscode/mcp.json`:

```json
{
  "inputs": [{
    "id": "ado_org",
    "type": "promptString",
    "description": "Azure DevOps organization name (e.g. 'contoso')"
  }],
  "servers": {
    "ado": {
      "type": "stdio",
      "command": "npx",
      "args": ["-y", "@azure-devops/mcp", "${input:ado_org}"]
    }
  }
}
```

### Configuration for Claude Desktop

Add to `claude_desktop_config.json`:

```json
{
  "mcpServers": {
    "azure-devops": {
      "command": "npx",
      "args": ["-y", "@azure-devops/mcp", "your-org-name"]
    }
  }
}
```

### Available Domains (Tool Groups)

Use `-d` flag to load specific domains:

```json
"args": ["-y", "@azure-devops/mcp", "org-name", "-d", "core", "work-items", "repositories"]
```

| Domain | Description |
|--------|-------------|
| `core` | Project-level information (recommended baseline) |
| `work` | Work management (sprints, iterations, teams) |
| `work-items` | Work item CRUD operations |
| `search` | Code and work item search |
| `test-plans` | Test plan management |
| `repositories` | Git repository operations |
| `wiki` | Wiki page management |
| `pipelines` | Build and release pipelines |
| `advanced-security` | Security scanning results |

### Example Capabilities
- "List my ADO projects"
- "List ADO Repos for 'Contoso'"
- "List my work items for project 'Contoso'"
- "Create a wiki page '/Architecture/Overview'"
- "List iterations for project 'Contoso'"

### Links
- GitHub: https://github.com/microsoft/azure-devops-mcp
- Docs: https://learn.microsoft.com/en-us/azure/devops/mcp-server/mcp-server-overview
- Blog: https://devblogs.microsoft.com/devops/azure-devops-mcp-server-public-preview/

### Limitations
- Only supports Azure DevOps Services (cloud), not on-premises Azure DevOps Server

---

## Approach 2: Community MCP Server (Tiberriver256)

### Overview
A community-built alternative that provides more authentication options and can serve as a customization base.

### Key Features
- **Multiple auth methods** - PAT, Azure Identity (DefaultAzureCredential), Azure CLI
- **Service principal support** - Good for automation scenarios
- **Modular architecture** - Easier to extend/customize
- **MIT licensed** - Full source available

### Installation

```bash
npx @tiberriver256/mcp-server-azure-devops
```

### Configuration

Environment variables:

```bash
# Required
AZURE_DEVOPS_ORG_URL=https://dev.azure.com/your-org

# Authentication (choose one method)
AZURE_DEVOPS_AUTH_METHOD=pat  # or 'azure-identity' or 'azure-cli'

# For PAT authentication
AZURE_DEVOPS_PAT=your-personal-access-token

# For Service Principal (azure-identity)
AZURE_CLIENT_ID=your-client-id
AZURE_TENANT_ID=your-tenant-id
AZURE_CLIENT_SECRET=your-client-secret
```

### Claude Desktop Configuration

```json
{
  "mcpServers": {
    "azure-devops": {
      "command": "npx",
      "args": ["@tiberriver256/mcp-server-azure-devops"],
      "env": {
        "AZURE_DEVOPS_ORG_URL": "https://dev.azure.com/your-org",
        "AZURE_DEVOPS_AUTH_METHOD": "pat",
        "AZURE_DEVOPS_PAT": "your-pat-token"
      }
    }
  }
}
```

### Available Capabilities
- **Projects**: List, get details
- **Repositories**: List, get files, browse trees, create branches, commits
- **Work Items**: Create, update, search, link
- **Pull Requests**: Create, comment, update status, view changes
- **Pipelines**: Run, get artifacts, view logs, timelines
- **Wiki**: Get pages, search content
- **Search**: Code search, work item search

### Links
- GitHub: https://github.com/Tiberriver256/mcp-server-azure-devops
- NPM: https://www.npmjs.com/package/@tiberriver256/mcp-server-azure-devops

---

## Approach 3: Build Custom MCP Server

### When to Build Custom
- Need specialized tools not in existing servers
- Want to integrate with additional systems alongside ADO
- Learning MCP protocol/architecture
- Need on-premises Azure DevOps Server support

### Technology Choices

#### TypeScript (Recommended)
- Strong typing for tool definitions
- Fast iteration with Node.js ecosystem
- Official SDK well-maintained

```bash
npm install @modelcontextprotocol/server zod
```

#### Python
- FastMCP provides high-level abstractions
- Good for data science/ML integration

```bash
pip install mcp
```

### MCP Core Concepts

1. **Tools** - Functions the LLM can invoke (side effects, API calls)
2. **Resources** - Read-only data exposed to clients
3. **Prompts** - Reusable templates for consistent interactions
4. **Transports** - Communication layer (stdio, HTTP)

### Basic TypeScript Server Structure

```typescript
import { McpServer } from "@modelcontextprotocol/server";
import { StdioServerTransport } from "@modelcontextprotocol/server/transports";
import { z } from "zod";

const server = new McpServer({
  name: "azure-devops-custom",
  version: "1.0.0",
});

// Define a tool
server.tool(
  "list_projects",
  "List all projects in the Azure DevOps organization",
  {
    // Input schema (using Zod)
  },
  async (params) => {
    // Call Azure DevOps REST API
    // Return results
  }
);

// Connect transport
const transport = new StdioServerTransport();
await server.connect(transport);
```

### Azure DevOps REST API Reference

Base URL: `https://dev.azure.com/{organization}`

| Resource | Endpoint | Method |
|----------|----------|--------|
| Projects | `/_apis/projects` | GET |
| Repositories | `/{project}/_apis/git/repositories` | GET |
| Work Items | `/_apis/wit/workitems/{id}` | GET |
| Work Item Query | `/{project}/_apis/wit/wiql` | POST |
| Pull Requests | `/{project}/_apis/git/repositories/{repo}/pullrequests` | GET/POST |
| Pipelines | `/{project}/_apis/pipelines` | GET |
| Builds | `/{project}/_apis/build/builds` | GET |

API Version: `api-version=7.1`

### Authentication Options

1. **Personal Access Token (PAT)**
   ```typescript
   const headers = {
     'Authorization': `Basic ${Buffer.from(`:${pat}`).toString('base64')}`
   };
   ```

2. **Azure Identity (DefaultAzureCredential)**
   ```typescript
   import { DefaultAzureCredential } from "@azure/identity";
   const credential = new DefaultAzureCredential();
   const token = await credential.getToken("499b84ac-1321-427f-aa17-267ca6975798/.default");
   ```

3. **Azure CLI**
   ```bash
   az login
   az account get-access-token --resource 499b84ac-1321-427f-aa17-267ca6975798
   ```

### Project Structure Example

```
mcp-ado-custom/
├── package.json
├── tsconfig.json
├── src/
│   ├── index.ts           # Entry point, server setup
│   ├── auth/
│   │   └── credentials.ts # Authentication handling
│   ├── tools/
│   │   ├── projects.ts    # Project-related tools
│   │   ├── repos.ts       # Repository tools
│   │   ├── workitems.ts   # Work item tools
│   │   └── pullrequests.ts# PR tools
│   └── types/
│       └── ado.ts         # TypeScript types for ADO responses
```

### SDK Resources
- TypeScript SDK: https://github.com/modelcontextprotocol/typescript-sdk
- Python SDK: https://github.com/modelcontextprotocol/python-sdk
- MCP Specification: https://spec.modelcontextprotocol.io
- Example Servers: https://github.com/modelcontextprotocol/servers

---

## Comparison Matrix

| Feature | Microsoft Official | Community (Tiberriver256) | Custom Build |
|---------|-------------------|---------------------------|--------------|
| Setup Time | Minutes | Minutes | Days/Weeks |
| Maintenance | Microsoft | Community | You |
| Auth Options | Browser/MS Account | PAT, Azure Identity, CLI | Flexible |
| Customization | Limited | Fork & modify | Full control |
| On-Prem Support | No | Possible | Yes |
| Documentation | Excellent | Good | You write it |
| Production Ready | Yes | Yes | Depends |

---

## Quick Start Recommendation

For most users wanting to integrate Azure DevOps with Claude or other AI assistants:

```bash
# 1. Install (nothing to install, npx handles it)
# 2. Configure Claude Desktop (~/.config/claude/claude_desktop_config.json on Linux)

{
  "mcpServers": {
    "azure-devops": {
      "command": "npx",
      "args": ["-y", "@azure-devops/mcp", "your-organization-name", "-d", "core", "work-items", "repositories"]
    }
  }
}

# 3. Restart Claude Desktop
# 4. First tool use will prompt for Microsoft login
```

---

## Sources

- [Microsoft Azure DevOps MCP Server - GitHub](https://github.com/microsoft/azure-devops-mcp)
- [Azure DevOps MCP Server Overview - Microsoft Learn](https://learn.microsoft.com/en-us/azure/devops/mcp-server/mcp-server-overview)
- [Azure DevOps MCP Server GA Announcement - InfoQ](https://www.infoq.com/news/2025/11/microsoft-ado-mcp-server/)
- [Community MCP Server - GitHub](https://github.com/Tiberriver256/mcp-server-azure-devops)
- [MCP TypeScript SDK - GitHub](https://github.com/modelcontextprotocol/typescript-sdk)
- [Azure DevOps REST API - Microsoft Learn](https://learn.microsoft.com/en-us/rest/api/azure/devops/git/?view=azure-devops-rest-7.1)
- [Build MCP Servers with TypeScript - DEV Community](https://dev.to/shadid12/how-to-build-mcp-servers-with-typescript-sdk-1c28)
