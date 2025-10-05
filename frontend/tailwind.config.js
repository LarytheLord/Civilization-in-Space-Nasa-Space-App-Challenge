/** @type {import('tailwindcss').Config} */
module.exports = {
  content: [
    "./src/**/*.{js,jsx,ts,tsx}",
    "./public/index.html"
  ],
  theme: {
    extend: {
      colors: {
        // Professional NASA-grade color palette
        mission: {
          // Deep space backgrounds
          void: '#000000',
          deep: '#0a0e1a',
          dark: '#0f1419',
          panel: '#151b26',
          surface: '#1a2332',
          
          // Accent colors - subtle and professional
          cyan: {
            dim: '#1e3a5f',
            DEFAULT: '#2d5f8d',
            bright: '#3b82f6',
            glow: '#60a5fa',
          },
          blue: {
            dim: '#1e2a4a',
            DEFAULT: '#2563eb',
            bright: '#3b82f6',
            glow: '#60a5fa',
          },
          
          // Status colors - professional and clear
          success: {
            dim: '#064e3b',
            DEFAULT: '#059669',
            bright: '#10b981',
            glow: '#34d399',
          },
          warning: {
            dim: '#78350f',
            DEFAULT: '#d97706',
            bright: '#f59e0b',
            glow: '#fbbf24',
          },
          danger: {
            dim: '#7f1d1d',
            DEFAULT: '#dc2626',
            bright: '#ef4444',
            glow: '#f87171',
          },
          
          // Data visualization
          chart: {
            line1: '#3b82f6',
            line2: '#8b5cf6',
            line3: '#06b6d4',
            area1: 'rgba(59, 130, 246, 0.1)',
            area2: 'rgba(139, 92, 246, 0.1)',
          },
          
          // Text hierarchy
          text: {
            primary: '#e2e8f0',
            secondary: '#94a3b8',
            tertiary: '#64748b',
            dim: '#475569',
          },
          
          // Borders and dividers
          border: {
            subtle: 'rgba(148, 163, 184, 0.1)',
            DEFAULT: 'rgba(148, 163, 184, 0.2)',
            strong: 'rgba(148, 163, 184, 0.3)',
            bright: 'rgba(59, 130, 246, 0.4)',
          }
        }
      },
      fontFamily: {
        mono: ['Roboto Mono', 'SF Mono', 'Monaco', 'Consolas', 'monospace'],
        sans: ['Inter', 'system-ui', 'sans-serif'],
        display: ['Orbitron', 'sans-serif'],
      },
      animation: {
        'pulse-slow': 'pulse 3s cubic-bezier(0.4, 0, 0.6, 1) infinite',
        'spin-slow': 'spin 3s linear infinite',
        'glow': 'glow 2s ease-in-out infinite',
      },
      keyframes: {
        glow: {
          '0%, 100%': { opacity: '1' },
          '50%': { opacity: '0.5' },
        }
      },
      boxShadow: {
        'glow-sm': '0 0 10px rgba(59, 130, 246, 0.3)',
        'glow': '0 0 20px rgba(59, 130, 246, 0.4)',
        'glow-lg': '0 0 30px rgba(59, 130, 246, 0.5)',
        'panel': '0 4px 6px -1px rgba(0, 0, 0, 0.3), 0 2px 4px -1px rgba(0, 0, 0, 0.2)',
      }
    },
  },
  plugins: [],
}
